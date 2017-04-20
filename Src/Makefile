CC=gcc
CFLAGS=-Wall

test_queue: unity queue.c
	$(CC) $(CFLAGS) $(ADD) -DUNIT_TEST -o test_queue ./unity/src/unity.c queue.c
	./test_queue

unity:
	wget https://github.com/ThrowTheSwitch/Unity/archive/master.zip
	unzip master.zip
	mv Unity-master unity
	
clean:
	rm -r test_queue unity* master.zip *~
