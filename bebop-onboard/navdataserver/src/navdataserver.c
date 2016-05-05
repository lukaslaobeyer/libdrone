#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <memory.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/socket.h>
#include <netdb.h>

#define DEFAULT_BLACKBOX_FILE "/data/ftp/internal_000/Debug/current/blackbox/light_run_0"
#define MAX_FILENAME_LENGTH 128
#define HEADER_MAX_SIZE 4096*4
#define ERR_HEADER_TOO_LONG 1
#define UPDATE_INTERVAL_MS 25
#define DATA_MARKER "-- Data\n"
#define DEFAULT_UDP_PORT 56789
#define RECV_BUFSIZE 1024

int extractHeader(FILE *blackbox, char *header, int header_max_size, int *header_length)
{
    int i;
    char *data_str_location;

    for(i = 0; i < header_max_size - 1; i++)
    {
        header[i] = fgetc(blackbox);
        header[i+1] = 0;

        assert(header[i] != EOF);

        data_str_location = strstr(header, DATA_MARKER);
        if(data_str_location != NULL)
        {
            *header_length = (int) (data_str_location - header);
            return 0;
        }
    }

    return ERR_HEADER_TOO_LONG;
}

int extractMetadata(char *header, int *datakey_number, int *datakey_length)
{
    char *nentries_marker = "nentries:";
    char *datasize_marker = "datasize:";

    char *nentries_marker_loc = strstr(header, nentries_marker);
    char *datasize_marker_loc = strstr(header, datasize_marker);

    if(nentries_marker_loc == NULL || datasize_marker_loc == NULL)
    {
        return 1;
    }

    sscanf(nentries_marker_loc + strlen(nentries_marker), "%i", datakey_number);
    sscanf(datasize_marker_loc + strlen(datasize_marker), "%i", datakey_length);

    return 0;
}

int main(int argc, char *argv[])
{
    char blackbox_file[MAX_FILENAME_LENGTH] = DEFAULT_BLACKBOX_FILE;
    FILE *blackbox;
    char *header;
    int header_length = -1;
    int err;
    int datakey_number = -1;
    int datakey_length = -1;
    int packet_length = -1;
    int data_start_location;
    long current_data_size;
    int latest_navdata_packet_beginning;
    int navdata_socket;
    struct sockaddr_in navdata_addr;
    struct sockaddr_in navdata_receiver;
    socklen_t navdata_receiver_size = sizeof(navdata_receiver);
    char recv_buf[RECV_BUFSIZE];
    int recv_len;
    char *navdata_buf;

    if(argc == 2)
    {
        assert(strlen(argv[1]) <= MAX_FILENAME_LENGTH);
        strcpy(blackbox_file, argv[1]);
    }

    errno = 0;

    blackbox = fopen(blackbox_file, "rb");
    if(blackbox == NULL)
    {
        fprintf(stderr, "Error opening blackbox file: %s. Error number %i.\n", blackbox_file, errno);
        exit(1);
    }

    header = (char *) malloc(HEADER_MAX_SIZE);
    err = extractHeader(blackbox, header, HEADER_MAX_SIZE, &header_length);
    if(err != 0)
    {
        if(err == ERR_HEADER_TOO_LONG)
        {
            fprintf(stderr, "Blackbox header too long! Must be shorter than %i.", HEADER_MAX_SIZE);
        }
        else
        {
            fprintf(stderr, "Unknown error occurred extracting header!");
        }

        exit(1);
    }

    data_start_location = header_length + (int) strlen(DATA_MARKER);

    printf("Data starts at byte %i.\n", data_start_location);

    err = extractMetadata(header, &datakey_number, &datakey_length);
    if(err != 0)
    {
        fprintf(stderr, "Could not extract metadata from header!");
        exit(1);
    }

    packet_length = datakey_length * datakey_number;

    printf("Data composed of packets with %i %i-byte datakeys; packet length is %i.\n", datakey_number, datakey_length, packet_length);

    assert(datakey_length == sizeof(double));

    // Good to go. Open a UDP socket on DEFAULT_UDP_PORT
    navdata_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if(navdata_socket < 0)
    {
        fprintf(stderr, "Error creating UDP socket. Error number %i.\n", errno);
        fclose(blackbox);
        exit(1);
    }

    memset((char *)&navdata_addr, 0, sizeof(navdata_addr));
    navdata_addr.sin_family = AF_INET;
    navdata_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    navdata_addr.sin_port = htons(DEFAULT_UDP_PORT);

    if(bind(navdata_socket, (struct sockaddr *) &navdata_addr, sizeof(navdata_addr)) < 0)
    {
        fprintf(stderr, "Error binding UDP socket on port %i. Error number %i.\n", DEFAULT_UDP_PORT, errno);
        fclose(blackbox);
        exit(1);
    }

    printf("Waiting for packet on port %i.\n", DEFAULT_UDP_PORT);

    recv_len = recvfrom(navdata_socket, recv_buf, RECV_BUFSIZE, 0, (struct sockaddr *)&navdata_receiver, &navdata_receiver_size);

    printf("Got packet. Sending navdata!\n");

    navdata_buf = (char *) malloc(packet_length);

    while(1)
    {
        fseek(blackbox, 0L, SEEK_END);
        current_data_size = ftell(blackbox) - data_start_location;

        latest_navdata_packet_beginning = data_start_location + (current_data_size / packet_length) * packet_length - packet_length;
        fseek(blackbox, latest_navdata_packet_beginning, SEEK_SET);

        int read = fread(navdata_buf, packet_length, 1, blackbox);

        sendto(navdata_socket, navdata_buf, packet_length, 0, (struct sockaddr *) &navdata_receiver, sizeof(navdata_receiver));

        usleep(UPDATE_INTERVAL_MS * 1000);
    }

    fclose(blackbox);
    return 0;
}
