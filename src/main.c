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
#define MSG_SIZE 30 /* fila para armazenar até 10 mensagens (alinhada ao limite de 4 bytes) */

const struct device* stx = DEVICE_DT_GET(DT_NODELABEL(gpiob)); 

K_FIFO_DEFINE(fifo_dados);

struct lista_dados {
    void *fifo_reserved;  
    char sync;
    char stx;
    char id;
    char id_tamanho;
    char mensagem[8];
    char etx;
};
static struct lista_dados pacote = {
        .sync = 0x16,
        .stx = 0x2,
        .id = 0xAA,
        .etx = 0x3,
    };
struct lista_dados *envio;


/* fila para armazenar até 10 mensagens (alinhada ao limite de 4 bytes) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* buffer de recepção usado no callback da ISR UART */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

int validar_32_bits_para_8_bits(uint32_t buffer, uint8_t *resultado) {
    // Máscaras para isolar os bits centrais de cada grupo de 4 bits
    uint32_t mascara_c1 =  0x44444444; //0b01000100010001000100010001000100
    uint32_t mascara_c2 =  0x22222222; //0b00100010001000100010001000100010

    // Isola os bits centrais
    uint32_t bits_c1 = (buffer & mascara_c1) >> 2;  // Segundo bit de cada grupo
    uint32_t bits_c2 = (buffer & mascara_c2) >> 1;  // Terceiro bit de cada grupo

    // Verifica se os bits centrais são diferentes (condição inválida)
    uint32_t invalidos = bits_c1 ^ bits_c2;
    if (invalidos != 0) {
        // Se algum grupo tiver bits centrais diferentes (01 ou 10), retorne erro
        return -1;
    }

    // Compacta os bits centrais válidos
    uint32_t iguais = bits_c1; // bits_c1 e bits_c2 são iguais neste ponto
    *resultado = ((iguais >> 28) & 0x1) << 7 | // Grupo 1
                 ((iguais >> 24) & 0x1) << 6 | // Grupo 2
                 ((iguais >> 20) & 0x1) << 5 | // Grupo 3
                 ((iguais >> 16) & 0x1) << 4 | // Grupo 4
                 ((iguais >> 12) & 0x1) << 3 | // Grupo 5
                 ((iguais >> 8)  & 0x1) << 2 | // Grupo 6
                 ((iguais >> 4)  & 0x1) << 1 | // Grupo 7
                 ((iguais)  & 0x1);       // Grupo 8

    return 0; // Sucesso
}

/* Lê caracteres da UART até que o final da linha seja detectado. Depois, envia os dados para a fila de mensagens. */
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

/* Envia bit a bit da fifo */
void enviar_bits(uint8_t dado, int delay_ms) {
    for (int i = 0; i < 8; i++) {
        if ((dado & 0b10000000) == 0b10000000) {
            gpio_pin_set(stx, 0x1, 1); // Envia bit 1 para GPIO
        } else {
            gpio_pin_set(stx, 0x1, 0); // Envia bit 0 para GPIO
        }
        k_msleep(delay_ms);
        dado <<= 1; // Desloca para o próximo bit
    }
}

void bitabit() {
    //printk("-----Processando dados da FIFO-----\n");

    while ((envio = k_fifo_get(&fifo_dados, K_NO_WAIT)) != NULL) {
        
        
        // Enviar STX
        enviar_bits(envio->stx, TEMPO_ESCRITA);
        //printk("Enviei o STX\n");
        enviar_bits(envio->id,  TEMPO_ESCRITA);
        //printk("Enviei o Id\n");
        enviar_bits(envio->id_tamanho,  TEMPO_ESCRITA);
        //printk("Enviei o Id Tamanho\n");

        // Enviar Mensagem
        for (int n = 0; n < sizeof(envio->mensagem); n++) {
            int msg = envio->mensagem[n];
            if (msg == '\0') {
                break; // Parar ao encontrar o caractere nulo
            }
            //printk("Mensagem da FIFO: %d\n", msg);
            enviar_bits(msg, TEMPO_ESCRITA);
        }
    }

    //printk("--------------\n\n");
    gpio_pin_set(stx, 0x1, 0); // Garante que o último estado seja 0
}
void armazenar(char *buf) {
    int msg_len = strlen(buf);
    int j = 0;
    int n_resto = msg_len%8;
    int n_mensagens = msg_len/8;

    //printk("O quociente é %d\n", n_mensagens);
    //printk("O resto é %d\n", n_resto);

    if(n_mensagens!=0){
    for(int n=0; n < n_mensagens; n++){
        for (int i = 0; i < 8; i++) {
            pacote.mensagem[i] = buf[j];
            j++;
        }
        pacote.id_tamanho = 0x08;
        k_fifo_put(&fifo_dados, &pacote);
        //printk("FIFO printed \n");
		bitabit();
        memset(pacote.mensagem, 0, sizeof(pacote.mensagem));
    }
    }
    if(n_resto!=0){
        for (int i = 0; i < n_resto; i++) {
                pacote.mensagem[i] = buf[j];
                j++;
            }
            pacote.id_tamanho = n_resto;
            k_fifo_put(&fifo_dados, &pacote);
            //printk("FIFO printed \n");
            bitabit();
            memset(pacote.mensagem, 0, sizeof(pacote.mensagem));
    j=0;
    }
}

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

void leitura(void) {
    gpio_pin_configure(stx, 0x0, GPIO_INPUT);
    int value = 0;
    uint32_t buffer_teste = 0; 
    uint8_t resultado;
    int status = 0;
    char id_lido = 0;
    char tamanho_lido = 0;
    char mensagem_lida = 0;

   while (1)
   {
    value = gpio_pin_get(stx, 0x0);
    // Extrai os bits centrais e verifica
    status = validar_32_bits_para_8_bits(buffer_teste, &resultado);
    if (status == 0 && resultado!=0) {
        
        //printf("achei: %d\n", (resultado));
        
        if(resultado == 0x2){
        //printf("achei STX %c\n", (resultado));
            for(int i=0; i<32; i++){   
                value = gpio_pin_get(stx, 0x0);
                buffer_teste <<= 1;
                buffer_teste = (buffer_teste | value);
                k_msleep(TEMPO_LEITURA);
            }
            status = validar_32_bits_para_8_bits(buffer_teste, &resultado);
            id_lido = resultado;
            //printf("Id lido: %d\n", id_lido);
            for(int i=0; i<32; i++){   
                value = gpio_pin_get(stx, 0x0);
                buffer_teste <<= 1;
                buffer_teste = (buffer_teste | value);
                k_msleep(TEMPO_LEITURA);
            }
            status = validar_32_bits_para_8_bits(buffer_teste, &resultado);
            tamanho_lido = resultado;
            //printf("Tamanho lido: %d\n", tamanho_lido);

            for(int i = 0; i<tamanho_lido; i++){
                for(int i=0; i<32; i++){   
                value = gpio_pin_get(stx, 0x0);
                buffer_teste <<= 1;
                buffer_teste = (buffer_teste | value);
                k_msleep(TEMPO_LEITURA);
            }
            status = validar_32_bits_para_8_bits(buffer_teste, &resultado);
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
K_THREAD_DEFINE(leitura_id, MY_STACK_SIZE, leitura, NULL, NULL, NULL, MY_PRIORITY, 0, 0);