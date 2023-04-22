import io
import json

from models.map import Map


class MapBuilder:
    def __init__(self):
        pass

    @classmethod
    def from_file(cls, file_path):
        with io.open(file_path, 'r', encoding='utf8') as file:
            json_data = json.loads(file.read())
            map_data = json_data.get('map', {})

            return Map(width=map_data.get('width', 0.0),
                        length=map_data.get('length', 0.0),
                        plates=map_data.get('plates', {}), 
                        cherries=map_data.get('cherries', {}), 
                        aruco_tags=map_data.get('atuco-tag', {}))
