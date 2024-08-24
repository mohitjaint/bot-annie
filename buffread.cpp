#include <stdio.h>
#include <string.h>


int main()
{
    char i;
    char buff[100] = "";
    int index = 0;

    while (i != ']')
    {
        scanf("%c", &i);

        // printf("Length of char '%c' is: %lu\n", i, strlen(&i));

        if (i != '\n')
        {
            buff[index] = i;
            index++;
        }
        
    }

    buff[index] = '\0'; // Null-terminate the string
    printf("%s", buff);

    return 0;
}