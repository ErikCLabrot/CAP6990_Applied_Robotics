'''
MIT License

Copyright (c) 2020 Fabian Reister

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

import click
import cv2
from cv2 import aruco
import numpy as np
import yaml
import os

class MarkerFactory:

    @staticmethod
    def create_marker(size, marker_id, margin):
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

        # white background
        img = 255 * np.ones((size, size), dtype=np.uint8)
        img_marker = aruco.drawMarker(aruco_dict, marker_id, size - 2 * margin)

        # add marker centered
        img[margin:-margin, margin:-margin] = img_marker

        return img


class TileMap:
    _map: np.ndarray

    def __init__(self, tile_size):
        self._map = 255 * np.ones((4, 3, tile_size, tile_size), dtype=np.uint8)

    def set_tile(self, pos: tuple, img: np.ndarray):
        assert np.all(self._map[pos[0], pos[1]].shape == img.shape)
        self._map[pos[0], pos[1]] = img

    def get_map_image(self):
        """ Merges the tile map into a single image """

        img = np.concatenate(self._map, axis=-1)
        img = np.concatenate(img, axis=-2)

        img = img.T

        return img


@click.command()
@click.argument("path", type=click.Path(exists=True))
@click.option("--tile_size", type=int, default=100)
@click.option("--start_id", type=int, default=0, help="Starting ArUco marker ID")
def main(path, tile_size, start_id):
    margin = int(0.3 * tile_size)

    marker_factory = MarkerFactory()
    tile_map = TileMap(tile_size)

    order = ['left', 'bottom', 'front', 'top', 'back', 'right']

    ids = []

    marker_id = start_id
    for i in range(4):
        for j in range(3):
            if i != 1 and (j == 0 or j == 2):
                continue

            marker_img = marker_factory.create_marker(tile_size, marker_id, margin)
            tile_map.set_tile((i, j), marker_img)
            ids.append(marker_id)

            marker_id += 1

    tile_img = tile_map.get_map_image()

    tile_img_square = np.zeros((tile_size * 4, tile_size * 4))
    tile_img_square[:, (tile_size // 2):(-tile_size // 2)] = tile_img

    tile_img = cv2.flip(tile_img, 1) # Flip the image horizontally
    tile_img_square = cv2.flip(tile_img_square, 1) # Flip the square image horizontally

    cv2.imwrite(os.path.join(path, "marker_tile.png"), tile_img)
    cv2.imwrite(os.path.join(path, "marker_tiles_square.png"), tile_img_square)

    marker_config = dict(zip(order, ids))

    config = dict()
    config["aruco_dict"] = "6X6_250"
    config["markers"] = marker_config

    with open(os.path.join(path, "marker_info.yml"), "w") as yml_file:
        yaml.dump(config, yml_file)


if __name__ == '__main__':
    main()
