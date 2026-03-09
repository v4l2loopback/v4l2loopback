all: test_dqbuf test_output_dqbuf consumer producer

test_output_dqbuf: test_output_dqbuf.c
	$(CC) -o $@ $< -lpthread

consumer producer: common.h
