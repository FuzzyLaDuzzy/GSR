import socket
import ssl
import time

def cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, object_name):
    """
    Cliente L-SNMPvS para interagir com o agente.

    Args:
        endereco_agente (str): Endereço IP do agente L-SNMPvS.
        porta_agente (int): Porta do agente L-SNMPvS.
        caminho_certificado_cliente (str): Caminho para o certificado do cliente.
        caminho_chave_privada_cliente (str): Caminho para a chave privada do cliente.
        object_name (str): Nome do objeto para o qual solicitar o valor.
    """

    contexto_ssl_cliente = ssl.create_default_context(ssl.Purpose.SERVER_AUTH)
    contexto_ssl_cliente.load_cert_chain(certfile=caminho_certificado_cliente, keyfile=caminho_chave_privada_cliente)
    contexto_ssl_cliente.check_hostname = False  # Disable hostname check.
    contexto_ssl_cliente.verify_mode = ssl.CERT_NONE  # Disable cert verification.

    with socket.create_connection((endereco_agente, porta_agente)) as sock:
        with contexto_ssl_cliente.wrap_socket(sock, server_hostname=endereco_agente) as ssock:
            try:
                # Enviar requisição GET para o objeto
                ssock.send(f"GET {object_name}".encode())

                # Receber e imprimir a resposta do agente
                resposta = ssock.recv(1024).decode()
                print(f"Resposta do agente para {object_name}: {resposta}")

            except Exception as e:
                print(f"Erro ao comunicar com o agente: {e}")


def test_agent(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente):
    """
    Testa o agente L-SNMPvS, solicitando vários objetos.

    Args:
        endereco_agente (str): Endereço IP do agente L-SNMPvS.
        porta_agente (int): Porta do agente L-SNMPvS.
        caminho_certificado_cliente (str): Caminho para o certificado do cliente.
        caminho_chave_privada_cliente (str): Caminho para a chave privada do cliente.
    """

    print("Iniciando testes do agente L-SNMPvS...")

    # Testar objetos do dispositivo
    print("\nTestando objetos do dispositivo:")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "device.lMibId")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "device.id")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "device.type")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "device.beaconRate")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "device.nSensors")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "device.dateAndTime")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "device.upTime")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "device.opStatus")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "device.reset")

    # Testar objetos dos sensores
    print("\nTestando objetos dos sensores:")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.1.id")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.1.type")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.1.sampleValue")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.1.minValue")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.1.maxValue")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.1.lastSamplingTime")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.1.samplingRate")

    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.2.id")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.2.type")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.2.sampleValue")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.2.minValue")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.2.maxValue")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.2.lastSamplingTime")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "sensors.2.samplingRate")

    # Testar um objeto inexistente
    print("\nTestando objeto inexistente:")
    cliente_lsnmps(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente, "device.nonexistent")

    print("\nTestes do agente L-SNMPvS concluídos.")


# Exemplo de uso do cliente de teste
if __name__ == "__main__":
    endereco_agente = "127.0.0.1"
    porta_agente = 12345
    caminho_certificado_cliente = "/home/fuzzymind/Documents/GSR/projekt/certificado.pem"  # o mesmo certificado que o servidor
    caminho_chave_privada_cliente = "/home/fuzzymind/Documents/GSR/projekt/chave_privada.pem"  # a mesma chave que o servidor

    test_agent(endereco_agente, porta_agente, caminho_certificado_cliente, caminho_chave_privada_cliente)
