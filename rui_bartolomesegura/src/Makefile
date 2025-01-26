CC = g++
CFLAGS = -Wall -Wshadow -Wvla -g -c
EFLAGS = -g -o

.RECIPEPREFIX = >

rui_bartolomesegura: rui_bartolomesegura.o
>$(CC) -I /usr/include/eigen3 $(EFLAGS) rui_bartolomesegura rui_bartolomesegura.o

rui_bartolomesegura.o: rui_bartolomesegura.cpp
>$(CC) $(CFLAGS) -I /usr/include/eigen3 rui_bartolomesegura.cpp

clean:
>rm -f *.o rui_bartolomesegura
