roadRoiExtract: main.o roadRoiExtract.o errorNIETO.o MSAC.o lmmin.o
	g++ -o ./roadRoiExtract main.o roadRoiExtract.o errorNIETO.o MSAC.o lmmin.o `pkg-config --libs opencv` 
	rm *.o
lmmin.o: lmmin.c lmmin.h
	g++ -o lmmin.o -c lmmin.c 
MSAC.o: MSAC.cpp MSAC.h errorNIETO.h lmmin.h
	g++ -o MSAC.o -c MSAC.cpp `pkg-config --cflags opencv` 
errorNIETO.o: errorNIETO.cpp errorNIETO.h 
	g++ -o errorNIETO.o -c errorNIETO.cpp `pkg-config --cflags opencv` 
roadRoiExtract.o: roadRoiExtract.cpp roadRoiExtract.h
	g++ -o roadRoiExtract.o -c roadRoiExtract.cpp `pkg-config --cflags opencv`
main.o: main.cpp errorNIETO.h MSAC.h
	g++ -o main.o -c main.cpp `pkg-config --cflags opencv` 
clean:
	rm laneDetector *.o

