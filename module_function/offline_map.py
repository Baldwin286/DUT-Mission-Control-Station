import os
import math
import urllib.request
import time

def deg2num(lat_deg, lon_deg, zoom):
    """Transform from latitude/longitude to tile numbers at a given zoom level"""
    lat_rad = math.radians(lat_deg)
    n = 2.0 ** zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return (xtile, ytile)

def download_map(center_lat, center_lon, zoom_levels, grid_size=3):
    """
    Load satellite map tiles from Google Maps and save them locally.
    grid_size=3 -> load 3x3 tiles around the center (total 9 tiles per zoom level)
    """
    # module_function/offline_map.py 
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # MAVLINK_APP
    base_dir = os.path.dirname(current_dir)
    
    # create offline_map folder
    map_dir = os.path.join(base_dir, "offline_Map")
    
    print(f"Start load map to: {map_dir}")
    
    for z in zoom_levels:
        center_x, center_y = deg2num(center_lat, center_lon, z)
        
        # grid_size
        offset = grid_size // 2
        for i in range(-offset, offset + 1):
            for j in range(-offset, offset + 1):
                x = center_x + i
                y = center_y + j
                
                path = os.path.join(map_dir, str(z), str(x))
                os.makedirs(path, exist_ok=True)
                
                filepath = os.path.join(path, f"{y}.png")
                
                if not os.path.exists(filepath):
                    url = f"http://mt0.google.com/vt/lyrs=y&hl=en&x={x}&y={y}&z={z}"
                    try:
                        urllib.request.urlretrieve(url, filepath)
                        print(f"Loaded: Zoom {z} | X:{x} Y:{y}")
                        time.sleep(0.1) 
                    except Exception as e:
                        print(f"Error X:{x} Y:{y} - {e}")
                else:
                    print(f"Available: Zoom {z} | X:{x} Y:{y}")

if __name__ == "__main__":
    LAT = 16.0748
    LON = 108.1498
    
    download_map(LAT, LON, zoom_levels=[17, 18, 19, 20], grid_size=5)
    print("\nOFFLINE MAP DOWNLOAD COMPLETE!")