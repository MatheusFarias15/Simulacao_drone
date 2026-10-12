# Simulação 3D de Drone com Python

Projeto acadêmico de simulação visual de um drone quadricóptero seguindo uma trajetória tridimensional.

## O que o projeto faz
- Anima um drone simplificado em 3D.
- Executa uma trajetória espacial em formato de oito, variando X, Y e Z.
- Mostra os eixos locais do drone e a orientação (roll, pitch e yaw) estimada a partir da trajetória.
- Gera gráficos de posição, velocidade e orientação ao longo do tempo.
- Salva os gráficos em `outputs/`.

> **Limitação importante:** esta é uma simulação cinemática para visualização da trajetória. Ela não calcula forças, empuxo individual dos motores, bateria, vento ou estabilidade de controle. Portanto, não representa um simulador físico completo nem valida que um drone real conseguiria seguir a trajetória.

## Requisitos
- Python 3.10 ou superior
- pip
- Matplotlib e NumPy

## Instalação

Abra um terminal na pasta do projeto e execute:

```bash
python -m venv .venv
```

Ativação no Windows:
```powershell
.venv\Scripts\Activate.ps1
```

Se o PowerShell bloquear a ativação, pode executar diretamente:
```powershell
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe src\main.py
```

Ativação no macOS/Linux:
```bash
source .venv/bin/activate
```

Depois instale as dependências:
```bash
python -m pip install -r requirements.txt
```

## Executar

Na pasta raiz:

```bash
python src/main.py
```

Uma janela será aberta com a animação 3D. Ao fechar a janela, os gráficos serão salvos na pasta `outputs/`.

Para gerar somente os gráficos, sem a animação:
```bash
python src/main.py --plots-only
```

Para salvar a animação como GIF (requer Pillow):
```bash
python -m pip install pillow
python src/main.py --save-gif
```

## Estrutura

```text
simulacao_drone_python/
├── README.md
├── requirements.txt
├── src/
│   └── main.py
├── docs/
│   └── fundamentacao_matematica.md
└── outputs/
```

## Parâmetros da trajetória

No início de `src/main.py`, ajuste:
- `DURATION`: duração da trajetória em segundos.
- `FPS`: quadros por segundo da animação.
- `N_POINTS`: número de amostras usadas nos cálculos.
- `RADIUS`: escala horizontal da trajetória.
- `Z_BASE` e `Z_AMPLITUDE`: altura média e variação vertical.

## Roteiro sugerido para apresentação
1. Objetivo e escopo.
2. Sistema de coordenadas global e local.
3. Equações da trajetória.
4. Cálculo numérico da velocidade.
5. Orientação estimada a partir da direção do movimento.
6. Demonstração da animação 3D.
7. Gráficos e análise.
8. Limitações e possíveis evoluções: modelo dinâmico, controle PID, motores e perturbações.

## Resultados esperados
- `outputs/posicao_3d.png`
- `outputs/posicoes_tempo.png`
- `outputs/velocidade_tempo.png`
- `outputs/orientacao_tempo.png`


