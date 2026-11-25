# Testes unitarios no PC: compila os modulos independentes de hardware
# do firmware com o gcc nativo.
SRC_FW = ../../firmware/src
CFLAGS = -std=gnu11 -Wall -Wextra -g -I$(SRC_FW) -fsanitize=address,undefined

test: test_controle
	./test_controle

test_controle: test_controle.c $(SRC_FW)/controle.c $(SRC_FW)/params.c \
               $(SRC_FW)/protocolo.c $(SRC_FW)/uart_fmt.c $(SRC_FW)/*.h
	$(CC) $(CFLAGS) -o $@ test_controle.c $(SRC_FW)/controle.c \
	    $(SRC_FW)/params.c $(SRC_FW)/protocolo.c $(SRC_FW)/uart_fmt.c

clean:
	rm -f test_controle

.PHONY: test clean
