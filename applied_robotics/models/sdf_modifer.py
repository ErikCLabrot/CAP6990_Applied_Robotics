import os
import xml.etree.ElementTree as ET

def update_sdf(directory):
    """Updates the model name, URI, and visual name in all model.sdf files inside aruco_box_(n) folders."""
    
    # Loop through all aruco_box_(n) directories
    for folder in os.listdir(directory):
        if folder.startswith("aruco_box_") and os.path.isdir(os.path.join(directory, folder)):
            sdf_path = os.path.join(directory, folder, "model.sdf")
            
            if not os.path.exists(sdf_path):
                print(f"Skipping {folder}, no model.sdf found.")
                continue
            
            # Parse the SDF XML
            tree = ET.parse(sdf_path)
            root = tree.getroot()
            
            # Update model name
            model_element = root.find("model")
            if model_element is not None:
                model_element.set("name", folder)  # Set the name attribute to folder name
            
            # Update URI inside <mesh> tag
            for mesh in root.findall(".//mesh/uri"):
                mesh.text = f"model://{folder}/model.dae"

            # Update visual name
            for visual in root.findall(".//visual"):
                visual.set("name", f"{folder}_mesh")  # Update visual name
            
            # Write back to file
            tree.write(sdf_path, encoding="utf-8", xml_declaration=True)
            print(f"Updated {sdf_path}")

if __name__ == "__main__":
    base_directory = os.getcwd()  # Change this if needed
    update_sdf(base_directory)
