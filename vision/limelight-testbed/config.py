import json
from pathlib import Path

def load_config_file(filename="config"):
    # load in config data/files
    with (Path(__file__).parent / f"data/json/{filename}.json").open() as f:
        config_data = json.load(f)
    return config_data
