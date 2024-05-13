#include "Reles.hpp"

void taskReles(void *params)
{
    ModuloRele Reles(2, 15, 5, 4, false);

    while(true)
    {
        for (int i = 0; i < N_RELES; i++)
        {
            Reles.toggle(i);
            vTaskDelay( 500 / portTICK_PERIOD_MS ); 

        }
    }
}