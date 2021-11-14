#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <string.h>
#include <regex.h>

#ifdef _WIN32
  #include <windows.h>
#else
  #include <fcntl.h>
  #include <termios.h>
  #include <dirent.h>
  #include <sys/types.h>
  #include <sys/stat.h>
  #include <errno.h>
  #include <sys/ioctl.h>
#endif

// Function defs
int find_ports(char names[][32]);

#ifdef _WIN32
  HANDLE open_serial_port(const char *, uint32_t);
  int write_port(HANDLE, uint8_t *, size_t);
  char getch();
#else
  int open_serial_port(const char *, uint32_t);
  int write_port(int, char *, size_t);
  char getch();
#endif

int main(int argc, char *argv[]) {
  int baud_rate = 9600;

  char opt;
  while((opt = getopt(argc, argv, ":b:")) != -1) {
    switch(opt) {
      case 'b':
        baud_rate = atoi(optarg);
        break;
    }
  }

  // Validate baud rate
  if (!(baud_rate == 4800 || baud_rate == 9600 || baud_rate == 19200 || baud_rate == 38400 || baud_rate == 115200)) {
    printf("Invalid baud rate supplied, switching to default of 9600.\n");
    baud_rate = 9600;
  }

  char names[256][32];
  int n = find_ports(names);

  int device_i = 0;
  if (n > 1) {
    printf("Please select the Arduino's port:\n");
    int i;
    for (i = 0; i < n; i++) {
      printf("%d) %s\n", i, names[i]);
    }

    do {
      if (device_i >= n) {
        printf("Invalid input!\n");
      }

      scanf("%d", &device_i);
    } while(device_i >= n);
  } else {
    printf("There is only one port available.\n");
  }

  printf("Talking to Arduino on %s with baud rate %d.\n", names[device_i], baud_rate);

  #ifdef _WIN32
    HANDLE port = open_serial_port(names[device_i], 9600);
    if (port == INVALID_HANDLE_VALUE) {
      printf("Unable to open serial port :(");
      return 0;
    }
  #else
    int port = open_serial_port(names[device_i], 9600);
    if (port == -1) {
      printf("Unable to open serial port :(");
      return 0;
    }
  #endif

  int note_count = 7;
  char notes[] = {'c', 'd', 'e', 'f', 'g', 'a', 'b'};
  int frequencies[] = {262, 294, 330, 349, 392, 440, 494};
  while(1) {
    char ch = getch();
    int frequency = -1;
    int i;
    for (i = 0; i < note_count; i++) {
      if (notes[i] == ch) {
        printf("Playing '%c'\n", ch);
        frequency = frequencies[i];
        break;
      }
    }

    if (frequency != -1) {
      char data[4] = {frequency >> 24, frequency >> 16, frequency >> 8, frequency};
      int n = write_port(port, data, 4);
      if (n != 4) {
        printf("Error writing data to the Arduino.\n");
      }
    }

    if (ch == 'q') {
      break;
    }
  }

  return 0;
}

int find_ports(char names[][32]) { // Linux needs longer names
  #ifdef _WIN32
    int i, n = 0;
    TCHAR lpTargetPath[5000];
    for (i = 0; i < 255; i++) { // Try each possible COM port one by one
      sprintf(names[n], "COM%d", i);
      DWORD test = QueryDosDevice(names[n], (LPSTR)lpTargetPath, 5000);
      if (test != 0) {
        n++;
      }
    }

    return n;
  #else
    DIR *dir = opendir("/dev");
    int n = 0;

    regex_t reg;
    if (regcomp(&reg, "tty\\.[a-zA-Z]+", REG_EXTENDED|REG_NOSUB) != 0) 
      return 0;

    if (dir != NULL) {
      while(1) {
        // Checking for the directory
        struct dirent *ent = readdir(dir);
        if(ent == NULL) {
          break;
        }

        // Parsing directory
        struct stat file_stat;
        stat(ent->d_name, &file_stat);
       
        if(regexec(&reg, ent->d_name, 0, NULL, 0) == 0) {
          strncpy(names[n], ent->d_name, 32);
          n++;
        }
      }

      regfree(&reg);
      closedir(dir);
      return n;
    } else {
      return 0;
    }
  #endif
}

#ifdef _WIN32
HANDLE open_serial_port(const char *device, uint32_t baud_rate) {
  HANDLE port = CreateFileA(device, GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  if (port == INVALID_HANDLE_VALUE) {
    return INVALID_HANDLE_VALUE;
  }

  BOOL success = FlushFileBuffers(port);
  if (!success) {
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  COMMTIMEOUTS timeouts = {0};
  timeouts.ReadIntervalTimeout = 0;
  timeouts.ReadTotalTimeoutConstant = 100;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 100;
  timeouts.WriteTotalTimeoutMultiplier = 0;

  success = SetCommTimeouts(port, &timeouts);
  if (!success) {
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  DCB state = {0};
  state.DCBlength = sizeof(DCB);
  state.BaudRate = baud_rate;
  state.ByteSize = 8;
  state.Parity = NOPARITY;
  state.StopBits = ONESTOPBIT;
  success = SetCommState(port, &state);
  if (!success) {
    CloseHandle(port);
    return INVALID_HANDLE_VALUE;
  }

  return port;
}

int write_port(HANDLE port, uint8_t *buf, size_t n) {
  DWORD write_n;
  BOOL ok = WriteFile(port, buf, n, &write_n, NULL);

  if (!ok) {
    return -1;
  }

  return write_n;
}

char getch() {
  HANDLE hStdin = GetStdHandle (STD_INPUT_HANDLE);
  INPUT_RECORD irInputRecord;
  DWORD dwEventsRead;
  CHAR cChar;

  while(ReadConsoleInputA (hStdin, &irInputRecord, 1, &dwEventsRead)) { /* Read key press */
    if (irInputRecord.EventType == KEY_EVENT
    	&&irInputRecord.Event.KeyEvent.wVirtualKeyCode != VK_SHIFT
    	&&irInputRecord.Event.KeyEvent.wVirtualKeyCode != VK_MENU
    	&&irInputRecord.Event.KeyEvent.wVirtualKeyCode != VK_CONTROL) {
        cChar = irInputRecord.Event.KeyEvent.uChar.AsciiChar;
	      ReadConsoleInputA (hStdin, &irInputRecord , 1, &dwEventsRead); /* Read key release */
	      return cChar;
    }
  }
  return EOF;
}
#else
int open_serial_port(const char *device, uint32_t baud_rate) {
  // Add back the /dev
  char path[80];
  sprintf(path, "/dev/%s", device);
  int fd = open(path, O_RDWR | O_NONBLOCK);
  if (fd == -1) {
    return -1;
  }
  
  // Some strange phenomenon where arduino resets on every command
  // Disabling DTR
  // int status;
  // ioctl(fd, TIOCMGET, &status);
  // status &= ~TIOCM_DTR;
  // ioctl(fd, TIOCMSET, &status);


  // Get the current configuration of the serial port.
  struct termios options;
  int result = tcgetattr(fd, &options);
  if (result != 0) {
    close(fd);
    return -1;
  }

  // Selecting baud rate
  speed_t brate;
  switch(baud_rate) {
    case 4800:
      brate = B4800;
      break;
    case 9600:
      brate = B9600;
      break;
    case 19200:
      brate = B19200;
      break;
    case 38400:
      brate = B38400;
      break;
    case 57600:
      brate = B57600;
      break;
    case 115200:
      brate = B115200;
      break;
    default:
      printf("\nUsing default baud rate of 9600");
      brate = B9600;
  }
  
  cfsetospeed(&options, brate);
  cfsetispeed(&options, brate);

  // Configuring 8N1
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  // No flow control
  options.c_cflag &= ~CRTSCTS;

  options.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  options.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  options.c_oflag &= ~OPOST; // make raw

  options.c_cc[VMIN]  = 0;
  options.c_cc[VTIME] = 0;

  tcsetattr(fd, TCSANOW, &options);
  
  if (tcsetattr(fd, TCSAFLUSH, &options) < 0) {
    close(fd);
    return -1;
  }

  return fd;
}

int write_port(int fd, char *buf, size_t n) {
  ssize_t wn = write(fd, buf, 4);
  return wn;
}

char getch() {
  // Init terminal
  struct termios old, tmp;
  tcgetattr(0, &old);
  tmp = old;
  tmp.c_lflag &= ~ICANON;
  tmp.c_lflag &= ~ECHO;
  tcsetattr(0, TCSANOW, &tmp);

  char ch = getchar();

  tcsetattr(0, TCSANOW, &old);

  return ch;
}
#endif
