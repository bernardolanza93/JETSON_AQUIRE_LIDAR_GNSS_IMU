from external_lis import *


def create_axes(length=1.0):
    # Asse X - Rosso
    box_x = o3d.geometry.TriangleMesh.create_box(width=length, height=0.02, depth=0.02)
    box_x.paint_uniform_color([1, 0, 0])  # Colore rosso per l'asse X
    box_x.translate([length / 2, 0, 0])  # Centrato sull'origine

    # Asse Y - Verde
    box_y = o3d.geometry.TriangleMesh.create_box(width=0.02, height=length, depth=0.02)
    box_y.paint_uniform_color([0, 1, 0])  # Colore verde per l'asse Y
    box_y.translate([0, length / 2, 0])  # Centrato sull'origine

    # Asse Z - Blu
    box_z = o3d.geometry.TriangleMesh.create_box(width=0.02, height=0.02, depth=length)
    box_z.paint_uniform_color([0, 0, 1])  # Colore blu per l'asse Z
    box_z.translate([0, 0, length / 2])  # Centrato sull'origine

    return [box_x, box_y, box_z]

def visualize_pc(pc, title):
    point_size = 2
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title)

    # Aggiungi le geometrie una alla volta
    vis.add_geometry(pc)

    axes = create_axes(length=1.0)
    for axis in axes:
        vis.add_geometry(axis)


    opt = vis.get_render_option()
    # Imposta il colore di sfondo (colore RGB normalizzato)
    opt.background_color = np.array([0.1, 0.1, 0.1])  # Colore nero
    opt.point_size = point_size
    vis.run()
    vis.destroy_window()

