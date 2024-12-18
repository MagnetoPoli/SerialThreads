#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <inttypes.h>
#include <string.h>

#define TEMPO_LEITURA 10
#define TEMPO_ESCRITA 40

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 7


#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define MSG_SIZE 100 /* fila para armazenar até 100 mensagens (alinhada ao limite de 4 bytes) */

const struct device* stx = DEVICE_DT_GET(DT_NODELABEL(gpiob)); 

K_FIFO_DEFINE(fifo_dados);//define a fifo para armazenar os pacotes

struct lista_dados { // define o pacote
    void *fifo_reserved;  
    char sync;
    char stx;
    char id;
    char id_tamanho;
    char mensagem[8];
    char etx;
};
static struct lista_dados pacote = { //cria o pacote
        .sync = 0x16,
        .stx = 0x2,
        .id = 0xAA,
        .etx = 0x3,
    };
struct lista_dados *envio;


/* fila para armazenar até 10 mensagens (alinhada ao limite de 4 bytes) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 30, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* buffer de recepção usado no callback da ISR UART */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

int validar_32_bits_para_8_bits(uint32_t buffer, uint8_t *resultado) {
    // mascaras para isolar os bits centrais de cada grupo de 4 bits
    uint32_t mascara_c1 =  0x44444444; //0b01000100010001000100010001000100
    uint32_t mascara_c2 =  0x22222222; //0b00100010001000100010001000100010

    // isola os bits centrais
    uint32_t bits_c1 = (buffer & mascara_c1) >> 2;  // segundo bit de cada grupo
    uint32_t bits_c2 = (buffer & mascara_c2) >> 1;  // terceiro bit de cada grupo

    // verifica se os bits centrais sao diferentes (condiçao invalida)
    uint32_t invalidos = bits_c1 ^ bits_c2;
    if (invalidos != 0) {
        // se algum grupo tiver bits centrais diferentes (01 ou 10), retorne erro
        return -1;
    }

    // compacta os bits centrais válidos
    uint32_t iguais = bits_c1; // bits_c1 e bits_c2 sao iguais neste ponto
    *resultado = ((iguais >> 28) & 0x1) << 7 | // grupo 1
                 ((iguais >> 24) & 0x1) << 6 | // grupo 2
                 ((iguais >> 20) & 0x1) << 5 | // grupo 3
                 ((iguais >> 16) & 0x1) << 4 | // grupo 4
                 ((iguais >> 12) & 0x1) << 3 | // grupo 5
                 ((iguais >> 8)  & 0x1) << 2 | // grupo 6
                 ((iguais >> 4)  & 0x1) << 1 | // grupo 7
                 ((iguais)  & 0x1);       // grupo 8

    return 0; // feito
}

/* ler caracteres da UART ate que o final da linha seja detectado. depois, envia os dados para a fila de mensagens. */
void serial_cb(const struct device *dev, void *user_data) {
    uint8_t c;

    if (!uart_irq_update(uart_dev)) {
        return;
    }

    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }

    /* lê até que a FIFO esteja vazia */
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if ((c == '\n' || c == '\r') && rx_buf_pos > 0) {
            /* termina a string */
            rx_buf[rx_buf_pos] = '\0';

            /* se a fila estiver cheia, a mensagem é descartada silenciosamente */
            k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

            /* reseta o buffer (ele foi copiado para a msgq) */
            rx_buf_pos = 0;
        } else if (rx_buf_pos < (sizeof(rx_buf) - 1)) {
            rx_buf[rx_buf_pos++] = c;
        }
        /* caso contrário: caracteres além do tamanho do buffer são descartados */
    }
}

/* envia bit a bit da fifo */
void enviar_bits(uint8_t dado, int delay_ms) {
    for (int i = 0; i < 8; i++) {
        if ((dado & 0b10000000) == 0b10000000) { //verifica com a mascara se é 1  ou 0 no bit mais significativo
            gpio_pin_set(stx, 0x1, 1); // envia bit 1 para GPIO
        } else {
            gpio_pin_set(stx, 0x1, 0); // envia bit 0 para GPIO
        }
        k_msleep(delay_ms);
        dado <<= 1; // desloca para o próximo bit
    }
}

void bitabit() {
    //printk("-----Processando dados da FIFO-----\n");

    while ((envio = k_fifo_get(&fifo_dados, K_NO_WAIT)) != NULL) {
        
        
        
        enviar_bits(envio->stx, TEMPO_ESCRITA);// enviar STX
        //printk("Enviei o STX\n");
        enviar_bits(envio->id,  TEMPO_ESCRITA);// enviar ID
        //printk("Enviei o Id\n");
        enviar_bits(envio->id_tamanho,  TEMPO_ESCRITA);// enviar tamanho da mensagem a ser enviada
        //printk("Enviei o Id Tamanho\n");

        
        for (int n = 0; n < sizeof(envio->mensagem); n++) {// enviar caracteres da mensagem
            int msg = envio->mensagem[n];
            if (msg == '\0') {
                break; // parar ao encontrar o caractere vazio
            }
            //printk("Mensagem da FIFO: %d\n", msg);
            enviar_bits(msg, TEMPO_ESCRITA);
        }
    }

    //printk("--------------\n\n");
    gpio_pin_set(stx, 0x1, 0); // garante que o ultimo estado seja 0 para nao flutuar
}

//armazena a mensagem lida no terminal em uma fifo
void armazenar(char *buf) {
    int msg_len = strlen(buf); //verifica o tamanho da mensagem lida no terminal
    int j = 0; //variavel que soma ate o tamanho da mensagem
    int n_mensagens = msg_len/8; //faz o quociente da divisao por 8 para decobrir quantas mensagens de 8 serao enviadas
    int n_resto = msg_len%8;  //faz o resto da divisao por 8 para descobrir o tamanho da ultima mensagem
    

    //printk("O quociente é %d\n", n_mensagens);
    //printk("O resto é %d\n", n_resto);

    if(n_mensagens!=0){
    for(int n=0; n < n_mensagens; n++){ //coloca cada mensagem num pacote e adiciona fifo
        for (int i = 0; i < 8; i++) {
            pacote.mensagem[i] = buf[j]; // adiciona cada byte em uma posição da mensagem 
            j++;
        }
        pacote.id_tamanho = 0x08; //tamanho da mensagem é 8
        k_fifo_put(&fifo_dados, &pacote); //adicionar na fifo o pacote criado
        //printk("FIFO printed \n");
		bitabit(); //enviar o pacote bit a bit
        memset(pacote.mensagem, 0, sizeof(pacote.mensagem)); // zera o pacote
    }
    }
    if(n_resto!=0){
        for (int i = 0; i < n_resto; i++) {
                pacote.mensagem[i] = buf[j];// adiciona cada byte em uma posição da mensagem
                j++;
            }
            pacote.id_tamanho = n_resto; //tamanho da mensagem é o resto
            k_fifo_put(&fifo_dados, &pacote); //adicionar na fifo o pacote criado
            //printk("FIFO printed \n");
            bitabit(); //enviar o pacote bit a bit
            memset(pacote.mensagem, 0, sizeof(pacote.mensagem)); // zera o pacote
    j=0;
    }
}

//pega dados do terminal
void pega_dados(void) {
    char tx_buf[100];

    if (!device_is_ready(uart_dev)) {
        printk("Dispositivo UART não encontrado!");
        return;
    }

    /* configura a interrupção e o callback para receber dados */
    int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

    if (ret < 0) {
        if (ret == -ENOTSUP) {
            printk("Suporte à API UART acionada por interrupção não habilitado\n");
        } else if (ret == -ENOSYS) {
            printk("O dispositivo UART não suporta a API acionada por interrupção\n");
        } else {
            printk("Erro ao definir o callback da UART: %d\n", ret);
        }
        return;
    }
    uart_irq_rx_enable(uart_dev);

    /* espera indefinidamente por entrada do usuário */
    while (k_msgq_get(&uart_msgq, &tx_buf, K_FOREVER) == 0) {
        //printk("Achei a mensagem\n");
        //printk("txbuff: %s\n", tx_buf);
		armazenar(tx_buf);
	
    }
}



// faz a leitura do gpio e verifica bytes validos
void rx(void) {
    gpio_pin_configure(stx, 0x0, GPIO_INPUT);
    gpio_pin_configure(stx, 0x1, GPIO_OUTPUT);
    gpio_pin_set(stx, 0x1, 0); // Garante que o último estado seja 0
    int value = 0;
    uint32_t buffer_teste = 0; //buffer de 32 bit
    uint8_t resultado; //buffer de 8 bit extraido dos bits centrais do de 32
    int status = 0; 
    char id_lido = 0;
    char tamanho_lido = 0;
    char mensagem_lida = 0;

   while (1)
   {
    value = gpio_pin_get(stx, 0x0);
    // Extrai os bits centrais e verifica
    status = validar_32_bits_para_8_bits(buffer_teste, &resultado); //32 bits válido ou não, se sim, retorna o valor para 8 bit
    if (status == 0 && resultado!=0) {
        
        //printf("achei: %d\n", (resultado));
        
        if(resultado == 0x2){ //resultado == STX
            buffer_teste <<= 1;
            buffer_teste = (buffer_teste | value);
        //printf("achei STX %c\n", (resultado));
            for(int i=0; i<32; i++){   
                value = gpio_pin_get(stx, 0x0);
                buffer_teste <<= 1;
                buffer_teste = (buffer_teste | value);
                k_msleep(TEMPO_LEITURA);
            }   
            status = validar_32_bits_para_8_bits(buffer_teste, &resultado);//32 bits válido ou não, se sim, retorna o valor para 8 bit
            id_lido = resultado;
            //printf("Id lido: %d\n", id_lido);
            for(int i=0; i<32; i++){   
                value = gpio_pin_get(stx, 0x0);
                buffer_teste <<= 1;
                buffer_teste = (buffer_teste | value);
                k_msleep(TEMPO_LEITURA);
            }
            status = validar_32_bits_para_8_bits(buffer_teste, &resultado);//32 bits válido ou não, se sim, retorna o valor para 8 bit
            tamanho_lido = resultado;
            //printf("Tamanho lido: %d\n", tamanho_lido);

            for(int i = 0; i<tamanho_lido; i++){
                for(int i=0; i<32; i++){   
                    value = gpio_pin_get(stx, 0x0);
                    buffer_teste <<= 1;
                    buffer_teste = (buffer_teste | value);
                    k_msleep(TEMPO_LEITURA);
                }
                status = validar_32_bits_para_8_bits(buffer_teste, &resultado);//32 bits válido ou não, se sim, retorna o valor para 8 bit
                mensagem_lida = resultado;
                printf("%c", mensagem_lida);
            }
            //printf("\n");

        }

    } 
    buffer_teste <<= 1;
    buffer_teste = (buffer_teste | value);
    k_msleep(TEMPO_LEITURA);
    


   }
   
    
}

/* Primeira thread le o que voce digita no teclado ate 7 bits (id) monta o pacote e trasnmite para a FIFO */
K_THREAD_DEFINE(pega_dados_id, MY_STACK_SIZE, pega_dados, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
K_THREAD_DEFINE(rx_id, MY_STACK_SIZE, rx, NULL, NULL, NULL, MY_PRIORITY, 0, 0);