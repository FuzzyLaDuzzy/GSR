import random
import time

class SensorVirtual:
    def __init__(self, nome, valor_min, valor_max, funcao_geracao, ritmo, precisao, ruido):
        self.nome = nome
        self.valor_min = valor_min
        self.valor_max = valor_max
        self.funcao_geracao = funcao_geracao
        self.ritmo = ritmo
        self.precisao = precisao
        self.ruido = ruido
        self.valor_atual = self.gerar_valor()

    def gerar_valor(self):
        if self.funcao_geracao == "aleatorio":
            valor = random.uniform(self.valor_min, self.valor_max)
        # Adicionar outras funções de geração aqui
        valor += random.uniform(-self.ruido, self.ruido)
        return round(valor, self.precisao)

    def obter_valor(self):
        self.valor_atual = self.gerar_valor()
        return self.valor_atual

# Exemplo de uso
sensor_temperatura = SensorVirtual("Temperatura", 20, 30, "aleatorio", 1, 1, 0.5)

while True:
    print(f"{sensor_temperatura.nome}: {sensor_temperatura.obter_valor()}")
    time.sleep(sensor_temperatura.ritmo)