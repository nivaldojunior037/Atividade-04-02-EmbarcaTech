# Atividade 29-01

Este é o repositório que armazena a tarefa solicitada no dia 03/02. Todos os arquivos necessários à execução já foram criados, de modo que basta seguir as instruções abaixo para executá-lo em seu dispositivo.

## Como Usar

1. Para acessar a atividade aramzenada, clone este repositório para seu dispositivo executando o seguinte comando no terminal de um ambiente adequado, como o VS Code, após criar um repositório local: 
git clone https://github.com/nivaldojunior037/Atividade-04-02-EmbarcaTech

2. Após isso, abra a pasta que contém os arquivos clonados em um ambiente como o VS Code e compile o código existente para que os demais arquivos necessários sejam criados em seu dispositivo

3. Ao fim da compilação, será gerado um arquivo .uf2 na pasta build do seu repositório. Esse arquivo deve ser copiado para a BitDogLab em modo BOOTSEL para que ele seja corretamente executado. 

4. Abra o monitor serial do VS Code ou outro que você possua e selecione a entrada serial à qual sua placa foi conectada (caso não saiba qual é, consulte os dispositivos conectados nas configurações do seu computador). Selecione a taxa de 115200 e inicie o monitoramento.

### Como Executar o Código

1. Esse código foi projetado para solicitar que o caractere seja inserido no terminal serial apenas após a realização de 4 cliques nos botões A ou B. Toda alteração (inserção de um caractere ou aperto de um botão) é noticiada no monitor serial e no display da BitDogLab. 

2. É possível inserir o caractere mesmo assim, mas ele só será mostrado no monitor e no display ao fim da contagem decrescente de 4 apertos dos botões. 

3. Os botões A e B ligam e desligam os LEDs verde e azul do conjunto RGB, respectivamente. Se o contador de apertos estiver em 0, um aperto em um deles é noticiado no monitor e no display, mas o estado do led não se altera até que uma letra seja inserida.

4. Se o caractere inserido for um número, surgirá na matriz de LEDs 5x5 um desenho do número, cada número de uma cor diferente. 

5. Quando o caractere for inserido, o contador volta para 4 e os botões podem alternar o estado dos LEDs RGB novamente. 

#### Link do vídeo

Segue o link do Drive com o vídeo onde é demonstrada a utilização do código: https://drive.google.com/drive/folders/1OhOdlYhX695AWWljkEA79RVXTa0xO0dl?usp=sharing 