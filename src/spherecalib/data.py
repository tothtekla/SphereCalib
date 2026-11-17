"""Explicit JSON frame manifests; paths resolve relative to the manifest."""
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.io import loadmat
from .geometry import camera_matrix, points_array


def read_cloud(path):
    path = Path(path)
    if path.suffix.lower() == ".npy":
        X = np.load(path, allow_pickle=False)
    elif path.suffix.lower() in (".xyz", ".txt", ".csv"):
        X = np.loadtxt(path, delimiter="," if path.suffix.lower() == ".csv" else None)
    elif path.suffix.lower() == ".ply":
        X = read_ply(path)
    else:
        raise ValueError("Supported clouds: XYZ/TXT/CSV/NPY/PLY")
    return points_array(X[:, :3], 3, 4)


def read_ply(path):
    """Read XYZ from ASCII or binary PLY with scalar vertex properties.

    Vertex must be the first nonempty element. Faces after vertices are ignored.
    Unsupported layouts are rejected rather than guessed.
    """
    types = {"char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
             "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
             "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
             "float": "f4", "float32": "f4", "double": "f8", "float64": "f8"}
    with Path(path).open('rb') as stream:
        if stream.readline().strip() != b'ply':
            raise ValueError("Not a PLY file")
        count, properties, vertex, format_name = None, [], False, None
        for _ in range(10000):
            line = stream.readline()
            if not line:
                raise ValueError("Truncated PLY header")
            fields = line.decode('ascii').split()
            if not fields or fields[0] in ('comment', 'obj_info'):
                continue
            if fields[0] == 'end_header':
                break
            if fields[0] == 'format':
                format_name = fields[1]
            elif fields[0] == 'element':
                vertex = fields[1] == 'vertex'
                if vertex:
                    count = int(fields[2])
                elif count is None and int(fields[2]) > 0:
                    raise ValueError("PLY vertex must be first nonempty element")
            elif fields[0] == 'property' and vertex:
                if fields[1] not in types:
                    raise ValueError("Unsupported PLY vertex property")
                properties.append((fields[2], types[fields[1]]))
        else:
            raise ValueError("PLY header too long")
        names = [name for name, _ in properties]
        if count is None or count < 1 or not all(name in names for name in ('x', 'y', 'z')):
            raise ValueError("Missing PLY XYZ vertices")
        if format_name == 'ascii':
            rows = [stream.readline().decode('ascii').split() for _ in range(count)]
            if any(len(row) != len(properties) for row in rows):
                raise ValueError("Truncated or invalid ASCII PLY vertices")
            values = np.array(rows, float)
            return values[:, [names.index(name) for name in ('x', 'y', 'z')]]
        if format_name not in ('binary_little_endian', 'binary_big_endian'):
            raise ValueError("Unsupported PLY format")
        endian = '<' if format_name == 'binary_little_endian' else '>'
        dtype = np.dtype([(name, endian+kind) for name, kind in properties])
        raw = stream.read(count*dtype.itemsize)
        if len(raw) != count*dtype.itemsize:
            raise ValueError("Truncated binary PLY vertices")
        values = np.frombuffer(raw, dtype=dtype)
        return np.column_stack([values[name] for name in ('x', 'y', 'z')])


def load_manifest(path):
    path = Path(path)
    metadata = json.loads(path.read_text(encoding="utf-8"))
    if metadata.get("units") != "meters" or metadata.get("pixel_origin") != 0:
        raise ValueError("Manifest must declare units=meters and pixel_origin=0")
    K = camera_matrix(metadata["K"])
    images, clouds, rois = [], [], []
    for frame in metadata["frames"]:
        image = cv2.imread(str(path.parent / frame["image"]), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(f"Cannot load image {frame['image']}")
        images.append(image)
        clouds.append(read_cloud(path.parent / frame["cloud"]))
        rois.append(frame.get("roi"))
    return images, clouds, K, rois, metadata


def inspect_mat(path):
    """Legacy MAT metadata adapter; ignores obsolete MATLAB function workspaces."""
    names = ["imgDir", "imgNames", "scan3dDir", "scan3dNames", "fu", "fv", "u0", "v0",
             "rGT", "S0CamGT", "S0LidGT", "rotGT", "transGT"]
    return loadmat(path, variable_names=names, simplify_cells=True)


def load_bundled_example(directory, camera="C1", lidar="L1"):
    """Load repository examples by explicit camera/LiDAR prefixes.

    MATLAB string arrays are not required: filenames are matched by S index.
    Intrinsics shift by one pixel to account for MATLAB's one-based indexing.
    Returns original metadata separately; transform GT convention is not guessed.
    """
    import re
    directory = Path(directory)
    candidates = [directory / f"input{directory.name[-1]}{camera}.mat",
                  directory / f"input{directory.name[-1]}.mat"]
    mat = next((p for p in candidates if p.exists()), None)
    if mat is None:
        raise ValueError("Cannot find bundled example metadata")
    metadata = inspect_mat(mat)
    def scan(folder, prefix):
        found = {}
        for path in (directory / folder).iterdir():
            match = re.fullmatch(re.escape(prefix)+r"_S(\d+)\.(png|bmp|xyz)", path.name, re.IGNORECASE)
            if match:
                found[int(match[1])] = path
        return found
    image_paths, cloud_paths = scan("data2D", camera), scan("data3D", lidar)
    ids = sorted(image_paths.keys() & cloud_paths.keys())
    if len(ids) < 3:
        raise ValueError("Need at least three matched example frames")
    images = [cv2.imread(str(image_paths[i]), cv2.IMREAD_COLOR) for i in ids]
    if any(image is None for image in images):
        raise ValueError("Cannot read example image")
    clouds = [read_cloud(cloud_paths[i]) for i in ids]
    K = np.array([[metadata['fu'], 0, metadata['u0']-1],
                  [0, metadata['fv'], metadata['v0']-1], [0, 0, 1.]], float)
    return images, clouds, K, metadata, ids
