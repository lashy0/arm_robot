import json
import logging.config
import os


def setup_logging(config_path: str) -> None:
    """Sets up logging configuration from a JSON file.

    Args
    ----
    config_path : str
        The path to the JSON file that contains the logging configuration.
    
    Raises
    ------
    FileNotFoundError:
        If the configuration file does not exist at the specified path.
    
    JSONDecodeError:
        If the configuration file is not a valid JSON file.
    
    Exception:
        If any other unexpected error occurs during the setup process.
    """
    try:
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        
        with open(config_path, 'r') as f:
            config = json.load(f)
        
        logging.config.dictConfig(config)
        print(f"Logging setup completed successfully using configuration from {config_path}")
    
    except FileNotFoundError as e:
        print(f"File not found: {e}")
        raise
    
    except json.JSONDecodeError as e:
        print(f"Invalid JSON format in the configuration file {config_path}: {e}")
        raise

    except Exception as e:
        print(f"Error setting up logging: {e}")
        raise
