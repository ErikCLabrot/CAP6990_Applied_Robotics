import os
import subprocess
import click

@click.command()
@click.option("--n", type=int, required=True, help="Number of textures to generate")
@click.option("--start_id", type=int, default=0, help="Starting ArUco marker ID")
@click.option("--tile_size", type=int, default=100, help="Size of the ArUco markers")
def batch_generate(n, start_id, tile_size):
    base_dir = os.getcwd()

    for i in range(n):
        output_path = os.path.join(base_dir, f"aruco_box_{i}", "materials", "textures")
        os.makedirs(output_path, exist_ok=True)

        subprocess.run([
            "python3", "generate_aruco_texture.py", output_path,
            "--tile_size", str(tile_size),
            "--start_id", str(start_id + i * 6)
        ])

if __name__ == '__main__':
    batch_generate()
