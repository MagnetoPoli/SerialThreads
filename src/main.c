#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <inttypes.h>
#include <string.h>

#define MY_STACK_SIZE 1024
#define MY_PRIORITY 7
#define SLEEP_TIME_MS 1000

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)
#define MSG_SIZE 9 /* fila para armazenar até 10 mensagens (alinhada ao limite de 4 bytes) */

const struct device* stx = DEVICE_DT_GET(DT_NODELABEL(gpiob)); 


K_FIFO_DEFINE(fifo_dados);

int aux1 = 0, aux2 = 0;

struct lista_dados {
    void *fifo_reserved;  
    char sync;
    uint8_t id_tamanho;
    char mensagem[8];
};
static struct lista_dados pacote = {
        .sync = 0x16,
    };
struct lista_dados* envio;


/* fila para armazenar até 10 mensagens (alinhada ao limite de 4 bytes) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/* buffer de recepção usado no callback da ISR UART */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

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
void bitabit(char *buf) {
    gpio_pin_configure(stx, 0x12, GPIO_OUTPUT_ACTIVE); //define GPIO como output no pino 0x12 (PTB18)
    printk("-----Processando dados da FIFO-----\n");
    while ((envio = k_fifo_get(&fifo_dados, K_NO_WAIT)) != NULL) {
        printk("ID do pacote: %d\n", envio->id_tamanho);

        for (int n = 0; n < sizeof(envio->mensagem); n++) {
            int msg = envio->mensagem[n];
            if (msg == '\0') {
                break; // Pare ao encontrar o caractere nulo
            }
            printk("Mensagem da FIFO: %d\n", msg);
            
            printk("Enviando: ");
            for (int i = 0; i < 8; i++) {
                if ((msg & 0b10000000) == 0b10000000) { //shift dos bits
                    printk("1");
                    gpio_pin_set(stx, 0x12, 1); // envia bit 1 pra GPIO
                    
                } else {
                    printk("0");
                    gpio_pin_set(stx, 0x12, 0); // envia bit 0 pra GPIO
                    
                }
                msg <<= 1;
            }
            printk("\n");
        }
    }
	printk("--------------\n\n");
}

void armazenar(char *buf) {
    int msg_len = strlen(buf);
    printk("%d - ", msg_len);

    if (msg_len == 8) {
        uint8_t result = 0;
        for (int i = 0; i < msg_len; i++) {
            // Desloca o resultado à esquerda para dar espaço ao próximo bit
            result <<= 1;
            if (buf[i] == '1') {
                // Adiciona 1 no bit menos significativo
                result |= 1;
            }
        }
        pacote.id_tamanho = result;
        aux1++;
		printk("aux1\n");

    } else {
        for (int i = 0; i < msg_len; i++) {
            pacote.mensagem[i] = buf[i];
        }
        aux2++;
		printk("aux2\n");
    }

    if (aux1 == 1 && aux2 == 1) {
        k_fifo_put(&fifo_dados, &pacote);
        printk("FIFO printed \n");

		bitabit(buf);

        pacote.id_tamanho = 0;
        memset(pacote.mensagem, 0, sizeof(pacote.mensagem));
        aux1 = 0;
        aux2 = 0;
    } else if (aux1 > 1 || aux2 > 1) {
        aux1 = 0;
        aux2 = 0;
		printk("2 id's\n");
    }
}

void pega_dados(void) {
    char tx_buf[MSG_SIZE];

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
        printk("Achei a mensagem\n");
		armazenar(tx_buf);
	
    }
}


/* Primeira thread le o que voce digita no teclado ate 7 bits (id) monta o pacote e trasnmite para a FIFO */
K_THREAD_DEFINE(pega_dados_id, MY_STACK_SIZE, pega_dados, NULL, NULL, NULL, MY_PRIORITY, 0, 0);
