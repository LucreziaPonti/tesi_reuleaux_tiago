import h5py

# Sostituisci 'tuo_file.h5' con il percorso del tuo file
file_path = '/home/oem/catkin_ws/src/tesi_reuleaux_tiago/reuleaux/map_generation/maps/tiago_arm_0.15_reachability.h5'

with h5py.File(file_path, 'r') as f:
    # Stampa l'intestazione del file
    print("File header:")
    for key in f.keys():
        print(f"  {key}: {f[key].name}")

    # Stampa la risoluzione
    resolution = f['/Spheres/sphere_dataset'].attrs.get('Resolution', None)
    if resolution is not None:
        print(f"Risoluzione: {resolution}")

    def print_structure(name, obj):
        print(f"{name} (type: {type(obj)})")
        if isinstance(obj, h5py.Dataset):
            print(f"  Dataset shape: {obj.shape}")
            print(f"  Dataset dtype: {obj.dtype}")
            # Stampa gli attributi
            for attr in obj.attrs:
                print(f"    Attribute {attr}: {obj.attrs[attr]}")

    f.visititems(print_structure)