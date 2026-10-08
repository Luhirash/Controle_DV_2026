# 🚗 Driverless Vehicle Control – Simulink  

Este repositório contém a implementação de um **controle para DV (Driverless Vehicle)** em MATLAB/Simulink, utilizando o **Driving Scenario Designer** para criar cenários de teste e simular trajetórias.  

---

## 📌 Fluxo de Funcionamento  

1. **Criação do cenário**  
   - No *Driving Scenario Designer*, desenhe a pista desejada.  
   - Salve o cenário no arquivo:  
     ```
     simpleTrack.mat
     ```  

2. **Processamento da trajetória**  
   - Rode o script:  
     ```matlab
     prepareTrajectory.m
     ```  
   - Ele processa o `simpleTrack.mat` e gera o arquivo:  
     ```
     refPath.mat
     ```  
   - Esse arquivo contém os pontos de referência condensados da pista.  

3. **Simulação no modelo Bike**  
   - O modelo `Modelo_bike.slx` utiliza o `initFcn` para carregar automaticamente o `refPath.mat`.  
   - A simulação roda em 2D, exibindo o veículo seguindo a trajetória definida no cenário.  

---

## 📂 Estrutura do Repositório  

```bash
.
├── simpleTrack.mat       # Arquivo de cenário criado no Driving Scenario
├── prepareTrajectory.m   # Script que processa o cenário
├── refPath.mat           # Trajetória processada para referência
├── Modelo_bike.slx              # Modelo Simulink (bicycle model)
└── README.md             # Este arquivo
