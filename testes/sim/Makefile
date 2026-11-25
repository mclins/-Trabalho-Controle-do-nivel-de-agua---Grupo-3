# Simulacao software-in-the-loop: firmware real (simavr) + modelo do tanque
#
#   make            compila o simulador e roda todos os cenarios
#   make run-normal roda um cenario especifico
FW      = ../../firmware/build/nivel.elf
OUT     = ../../docs/resultados
CENARIOS = normal hist_a hist_b hist_c abastecimento sensor manual rele config

CFLAGS  = -O2 -Wall -Wextra -std=gnu11
LDLIBS  = -lsimavr -lelf -lm

all: tanque_sim $(addprefix run-,$(CENARIOS))

tanque_sim: tanque_sim.c
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

$(FW):
	$(MAKE) -C ../../firmware

run-%: tanque_sim $(FW)
	@mkdir -p $(OUT)
	./tanque_sim $(FW) $* $(OUT)
	@echo "cenario $* ok"

clean:
	rm -f tanque_sim

.PHONY: all clean
