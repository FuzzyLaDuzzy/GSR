import json
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.backends import default_backend

MASTER_KEY = b"master_key_32_bytes_1234567890ab"

def encrypt_secrets(data, output_file):
    """Encrypts configuration data using AES-256-ECB and writes it to a file."""
    try:
        data_bytes = json.dumps(data).encode('utf-8')
        padding_len = 16 - (len(data_bytes) % 16)
        padded_data = data_bytes + bytes([padding_len] * padding_len)
        cipher = Cipher(algorithms.AES(MASTER_KEY), modes.ECB(), backend=default_backend())
        encryptor = cipher.encryptor()
        encrypted_data = encryptor.update(padded_data) + encryptor.finalize()
        with open(output_file, 'wb') as f:
            f.write(encrypted_data)
        print(f"Encrypted secrets written to {output_file}")
    except Exception as e:
        print(f"Error encrypting secrets: {e}")

if __name__ == "__main__":
    """Generates encrypted secrets files for L-SNMPvS agent and monitor."""
    agent_secrets = {
        "agent_id": "AGENT001",
        "KA": "agent_secret_key_32_bytes_1234567890",
        "monitors": {
            "MONITOR1": "monitor_secret_key_32_bytes_1234567890"
        }
    }
    encrypt_secrets(agent_secrets, "agent_secrets.json")
    monitor_secrets = {
        "monitor_id": "MONITOR1",
        "KM": "monitor_secret_key_32_bytes_1234567890",
        "agent_id": "AGENT001",
        "KA": "agent_secret_key_32_bytes_1234567890"
    }
    encrypt_secrets(monitor_secrets, "monitor_secrets.json")