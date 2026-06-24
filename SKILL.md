---
name: franka-device-package
description: Create Franka Desk .device end-effector packages from ROS package meshes, URDF, and profile JSON
---

# Franka Device Package Creator

Create `.device` packages for Franka Desk End Effector Settings.

## Inputs

| Input | Description | Example |
|-------|-------------|---------|
| `profile_json` | End-effector profile JSON with inertial/TCP data | `Robotiq 2F85 Gripper.endeffector-profile.json` |
| `urdf_dir` | Directory containing URDF/xacro files from ROS package | `urdf/` |
| `meshes_dir` | Directory containing DAE visual meshes from ROS package | `meshes/visual/` |
| `device_name` | Output device name (kebab-case) | `robotiq-2f85` |

## Package Structure

```
device-name/
├── endeffector-config.json    ← built from profile_json
├── endeffector.urdf           ← built from URDF references
├── endeffector.svg            ← optional
└── meshes/visual/
    └── *.dae                  ← transformed copies from meshes_dir
```

## Workflow

### 1. Parse Profile JSON

The profile JSON contains inertial and TCP data:

```json
{
  "id": "rg",
  "name": "Robotiq 2F85 Gripper",
  "inertial": {
    "mass": 0.9,
    "centerOfMass": {"x": 0, "y": 0, "z": 0.057},
    "inertia": {"x11": 0.002768, "x22": 0.003149, "x33": 0.000564}
  },
  "tcp": {
    "rotation": {"pitch": 0, "roll": 0, "yaw": 0},
    "translation": {"x": 0, "y": 0, "z": 0.16}
  }
}
```

### 2. Parse URDF for Joint Transforms

Read xacro/URDF files to extract the kinematic tree. Build cumulative transforms for each link:

```python
import xml.etree.ElementTree as ET

def parse_urdf_joints(urdf_path):
    """Extract joint origins from URDF. Returns dict of {child_link: (parent, xyz, rpy)}."""
    tree = ET.parse(urdf_path)
    joints = {}
    for joint in tree.iter('joint'):
        name = joint.get('name')
        jtype = joint.get('type')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        origin = joint.find('origin')
        xyz = [0, 0, 0]
        rpy = [0, 0, 0]
        if origin is not None:
            xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
            rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
        joints[child] = {'parent': parent, 'type': jtype, 'xyz': xyz, 'rpy': rpy}
    return joints

def compute_cumulative_transforms(joints, root_link):
    """Walk from root to leaves, accumulating transforms."""
    transforms = {root_link: np.eye(4)}
    queue = [root_link]
    while queue:
        current = queue.pop(0)
        for child, info in joints.items():
            if info['parent'] == current:
                T = make_transform(info['xyz'], info['rpy'])
                transforms[child] = transforms[current] @ T
                queue.append(child)
    return transforms
```

### 3. Copy and Transform DAE Meshes

```bash
mkdir -p device-name/meshes/visual/
cp meshes_dir/*.dae device-name/meshes/visual/
```

Apply cumulative transforms to position arrays in each DAE (see transform_dae below).

### 4. Generate endeffector-config.json

Convert profile JSON to Franka config format:

```python
def profile_to_config(profile, tcp_z_offset=0):
    """Convert profile JSON to endeffector-config.json."""
    inertial = profile.get('inertial', {})
    tcp = profile.get('tcp', {})
    com = inertial.get('centerOfMass', {})
    inertia = inertial.get('inertia', {})
    trans = tcp.get('translation', {})
    rot = tcp.get('rotation', {})

    # Build rotation matrix from roll/pitch/yaw
    R = rotation_from_rpy(rot.get('roll', 0), rot.get('pitch', 0), rot.get('yaw', 0))

    # Column-major 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[0, 3] = trans.get('x', 0)
    T[1, 3] = trans.get('y', 0)
    T[2, 3] = trans.get('z', 0) + tcp_z_offset

    return {
        "id": profile.get('id', 'custom'),
        "name": profile.get('name', 'Custom End Effector'),
        "color": "#4A90D9",
        "mass": inertial.get('mass', 1.0),
        "centerOfMass": [com.get('x', 0), com.get('y', 0), com.get('z', 0)],
        "transformation": T.flatten().tolist(),
        "inertia": [
            inertia.get('x11', 0.001), inertia.get('x12', 0), inertia.get('x13', 0),
            inertia.get('x21', 0), inertia.get('x22', 0.001), inertia.get('x23', 0),
            inertia.get('x31', 0), inertia.get('x32', 0), inertia.get('x33', 0.001)
        ]
    }
```

### 5. Generate endeffector.urdf

Create URDF referencing all DAE files:

```python
def generate_urdf(robot_name, link_name, dae_files):
    """Generate simple URDF with visual elements for each DAE."""
    visuals = []
    for dae in dae_files:
        visuals.append(f'''    <visual>
      <geometry>
        <mesh filename="package://franka_description/meshes/visual/{dae}"/>
      </geometry>
    </visual>''')

    return f'''<?xml version="1.0" encoding="utf-8"?>
<robot name="{robot_name}">
  <link name="{link_name}">
{chr(10).join(visuals)}
  </link>
</robot>'''
```

### 6. DAE Processing Functions

#### Transform Vertices

```python
import numpy as np
from lxml import etree
import collada

NS = "http://www.collada.org/2005/11/COLLADASchema"
TAG = lambda local: f"{{{NS}}}{local}"

def transform_dae(src, out, T):
    """Apply 4x4 transform to position arrays, rotation to normals."""
    doc = collada.Collada(src)
    pos_ids, norm_ids = set(), set()
    for g in doc.geometries:
        for p in g.primitives:
            pos_ids.add(p.sources['VERTEX'][0][2].lstrip('#') + "-array")
            norm_ids.add(p.sources['NORMAL'][0][2].lstrip('#') + "-array")

    tree = etree.parse(src, etree.XMLParser(remove_blank_text=False))
    root = tree.getroot()
    R = T[:3, :3]

    for fa in root.iter(TAG("float_array")):
        fid = fa.get("id")
        vals = np.array([float(x) for x in fa.text.split()])
        n = len(vals) // 3
        if fid in pos_ids:
            pts = vals.reshape(n, 3)
            pts_h = np.hstack([pts, np.ones((n, 1))])
            fa.text = " ".join(f"{x}" for x in (T @ pts_h.T).T[:, :3].flatten())
        elif fid in norm_ids:
            norms = vals.reshape(n, 3)
            rotated = (R @ norms.T).T
            lens = np.linalg.norm(rotated, axis=1, keepdims=True)
            lens[lens == 0] = 1
            fa.text = " ".join(f"{x}" for x in (rotated / lens).flatten())

    tree.write(out, xml_declaration=True, encoding="utf-8")
```

#### Fix DAE Effects (Critical)

```python
def fix_dae_effects(dae_path):
    """Strip phong to minimal: emission, diffuse, index_of_refraction only."""
    doc = etree.parse(dae_path, etree.XMLParser(remove_blank_text=False))
    root = doc.getroot()

    for effect in root.iter(TAG("effect")):
        phong = effect.find(f".//{TAG('phong')}")
        if phong is None:
            continue

        emission = diffuse = ior = None
        for child in list(phong):
            tag = child.tag.replace(f"{{{NS}}}", "")
            if tag == "emission":
                c = child.find(TAG("color"))
                emission = c.text if c is not None else "0 0 0 1"
            elif tag == "diffuse":
                c = child.find(TAG("color"))
                diffuse = c.text if c is not None else "0.7 0.7 0.7 1"
            elif tag == "index_of_refraction":
                f = child.find(TAG("float"))
                ior = f.text if f is not None else "1"
            phong.remove(child)

        em = etree.SubElement(phong, TAG("emission"))
        etree.SubElement(em, TAG("color"), sid="emission").text = emission or "0 0 0 1"
        diff = etree.SubElement(phong, TAG("diffuse"))
        etree.SubElement(diff, TAG("color"), sid="diffuse").text = diffuse or "0.7 0.7 0.7 1"
        ior_el = etree.SubElement(phong, TAG("index_of_refraction"))
        etree.SubElement(ior_el, TAG("float"), sid="ior").text = ior or "1"

    doc.write(dae_path, xml_declaration=True, encoding="utf-8")
```

#### Remove Problematic Elements

```python
def clean_dae(dae_path):
    """Remove Camera, Light nodes and their libraries."""
    doc = etree.parse(dae_path, etree.XMLParser(remove_blank_text=False))
    root = doc.getroot()

    for tag in ["library_cameras", "library_lights"]:
        for lib in root.findall(TAG(tag)):
            root.remove(lib)

    for node in root.iter(TAG("node")):
        if node.get("id") in ["Camera", "Light"]:
            parent_map = {c: p for p in root.iter() for c in p}
            if node in parent_map:
                parent_map[node].remove(node)

    doc.write(dae_path, xml_declaration=True, encoding="utf-8")
```

### 7. Repack

```bash
du -sh device-name/  # Must be under 20MB
tar cvzf device-name.device -C device-name .
```

## Key Gotchas

- **Phong effects**: Only `emission`, `diffuse`, `index_of_refraction` — no `ambient`, `specular`, `shininess`
- **Color attributes**: Must have `sid` attribute: `<color sid="emission">`
- **Camera/Light nodes**: Remove from DAE — causes rendering issues
- **Multiple `<visual>`**: Franka may only render the first one — prefer separate DAE files with baked transforms
- **Source IDs**: pycollada returns IDs without `-array` suffix; XML `float_array` elements have `-array` suffix
- **Transform order**: Column-major 4x4 matrix in config's `transformation` array
- **xacro includes**: May need to resolve `$(find package)` paths manually or inline macros

## Example: Robotiq 2F85 Joint Transforms

From `robotiq_2f_85_macro.urdf.xacro`, cumulative transforms at joint=0:

| Part | Translation |
|---|---|
| robotiq_base | identity |
| ur_to_robotiq_adapter | [0, 0, 0.011] |
| left_knuckle | [0.0306, 0, 0.0549] |
| right_knuckle | [-0.0306, 0, 0.0549] |
| left_inner_knuckle | [0.0127, 0, 0.0614] |
| right_inner_knuckle | [-0.0127, 0, 0.0614] |
| left_finger | base→knuckle @ [0.0315, 0, -0.0038] |
| right_finger | mirror |
| left_finger_tip | base→knuckle→finger @ [0.0056, 0, 0.0472] |
| right_finger_tip | mirror |
