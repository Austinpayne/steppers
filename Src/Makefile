CC=gcc
CFLAGS=-Wall

test_queue: unity test_queue.c queue.c
	$(CC) $(CFLAGS) $(ADD) -DUNIT_TEST -o test_queue test_queue.c ./unity/src/unity.c queue.c
	./test_queue

unity:
	wget https://github.com/ThrowTheSwitch/Unity/archive/master.zip
	unzip master.zip
	mv Unity-master unity
	
clean:
	rm -r test_queue unity* master.zip *~
