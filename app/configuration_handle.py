import configparser
import os

# Percorso del file di configurazione relativo al file del codice
config_file_path = os.path.join(os.path.dirname(__file__), '..', 'configuration', 'config_localization.ini')


# Default configuration
# NOT CASE SENSITIVE!!!!!
default_config = {
    'IMU': {
        'print_raw': 1,
        'key2': 'value2'
    },
    'Section2': {
        'key3': 'value3',
        'key4': 'value4'
    }
}


# Funzione per creare un file di configurazione con le variabili di default
def create_config_file():
    config = configparser.ConfigParser()
    config.read_dict(default_config)

    with open(config_file_path, 'w') as configfile:
        config.write(configfile)


# Funzione per leggere il file di configurazione e restituire un dizionario
def read_config_file():
    if not os.path.exists(config_file_path):
        create_config_file()

    config = configparser.ConfigParser()
    config.read(config_file_path)

    config_dict = {}
    for section in config.sections():
        config_dict[section] = {}
        for key, value in config.items(section):
            config_dict[section][key] = value

    return config_dict


# Leggi il file di configurazione
config_data_localization = read_config_file()

