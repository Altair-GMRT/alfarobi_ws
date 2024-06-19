#ifndef COMMUNICATION_H_
#define COMMUNICATION_H_

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 11111




namespace robotis_op
{


class clientSocket{
public:

    int sock = 0, valread, client_fd;
	struct sockaddr_in serv_addr;
	//char msg[5] = "1000";
    char buffer[1024] = {0};
    int tryCount = 0;

    clientSocket(){}

    int init(){

        tryCount = 0;

        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            printf("\n Socket creation error \n");
            return -1;
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(PORT);

        // Convert IPv4 and IPv6 addresses from text to binary
        // form
        if (inet_pton(AF_INET, "192.168.0.131", &serv_addr.sin_addr)
            <= 0) {
            printf(
                "\nALAMAT IP TIDAK VALIDDDDD \n");
            return -1;
        }

        while ((!(client_fd
            = connect(sock, (struct sockaddr*)&serv_addr,
                    sizeof(serv_addr)))
            < 0)) {
            printf("\nKONEKSIII GAGALLLLLLLL \n");
            tryCount += 1;

            if(tryCount > 15)
                return -1;
            //return -1;
        }

        return 1;
    }

    void transmitMsg(char msg[]){
        send(sock, msg, strlen(msg), 0);

        if(msg[0] == 'A')
            std::cout << "ALFA SUDAH MENYALAAAAAAAAAAA!!!\n";

        else if(msg[0] = 'B')
            std::cout << "ALFA SUDAH NGEPASSS\n";

        close(client_fd);
    }

    bool receiveMsg(){
        valread = read(sock, buffer, 1024);
        
        if (buffer[0] == 'A'){
            std::cout << "KIIIICK STAAAAAAAAAAAAAAAAAAAAAAART!!!!!\n";
            close(client_fd);
            return true;
        }

        else if (buffer[0] == 'B'){
            std::cout << "REPEAAATTT!!!!!\n";
            close(client_fd);
            return true;
        }

        else{
            close(client_fd);
            return false;
        }

        close(client_fd);
        return false;
    }

    void shutdownSocket(){
        close(client_fd);
    }


};

class serverSocket{
public:

    int server_fd, new_socket, valread;
	struct sockaddr_in address;
	int opt = 1;
	int addrlen = sizeof(address);
	char buffer[1024] = { 0 };
    //char msg[5] = "1000";

    serverSocket(){}

    int init(){
        // Creating socket file descriptor
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0))
            == 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // Forcefully attaching socket to the port 11111
        if (setsockopt(server_fd, SOL_SOCKET,
                    SO_REUSEADDR | SO_REUSEPORT, &opt,
                    sizeof(opt))) {
            perror("setsockopt");
            exit(EXIT_FAILURE);
        }
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(PORT);

        // Forcefully attaching socket to the port 11111
        if (bind(server_fd, (struct sockaddr*)&address,
                sizeof(address))
            < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
        if (listen(server_fd, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }
        if ((new_socket
            = accept(server_fd, (struct sockaddr*)&address,
                    (socklen_t*)&addrlen))
            < 0) {
            perror("accept");
            exit(EXIT_FAILURE);
        }
        return 1;
    }

    bool receiveMsg(){
        valread = read(new_socket, buffer, 1024);

        if (buffer[0] == 'A'){
            std::cout << "ALFA SUDAH MENYALA!\n";
            close(new_socket);
            shutdown(server_fd, SHUT_RDWR);
            return true;
        }

        else if(buffer[0] == 'B'){
            std::cout << "ALFA SUDAH PASSING\n";
            close(new_socket);
            shutdown(server_fd, SHUT_RDWR);
            return true;
        }

        else{
            close(new_socket);
            shutdown(server_fd, SHUT_RDWR);
            return false;
        }
        close(new_socket);
        shutdown(server_fd, SHUT_RDWR);
        return true;
    }

    void transmitMsg(char msg[]){
        send(new_socket, msg, strlen(msg), 0);

        if(buffer[0] == 'A'){
            std::cout << "STAAAAAAAAAAAAAAART PLAY!\n";
        }

        else if(buffer[0] == 'B'){
            std::cout << "LETS REPEAAAAAAAAAAAT\n";
        }
    }

    void shutdownSocket(){
        close(new_socket);
        shutdown(server_fd, SHUT_RDWR);
    }


};

}


#endif