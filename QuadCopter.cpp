#include "quadcopter.h"
#include <unistd.h>

    

int totPitchError = 0;
int totRollError = 0;
int totHeightError = 0;
int prevPitchError = 0;
int prevRollError = 0;
int prevHeightError = 0;

//desired height,pitch,roll
uint8_t desHeight = 100;
int8_t desPitch = 0;
int8_t desRoll = 0;

const int dt = 2;

//proportional, integral, and derivative constants, configure these
int8_t pkPitch;
int8_t ikPitch;
int8_t dkPitch;
int8_t pkRoll;
int8_t ikRoll;
int8_t dkRoll;
int8_t pkHeight;
int8_t ikHeight;
int8_t dkHeight;

int pid(int8_t pk, int8_t ik, int8_t dk, int sp, int pv, int &totError, int &prevError);


int main()
{
    int8_t pitch;
    int8_t roll;
    uint8_t height;
    int flSet = 127, frSet = 127, blSet = 127, brSet = 127;

    while (1)
    {
        pitch = getPitch();
        int addedPitch = pid(pkPitch,ikPitch,dkPitch,desPitch,pitch,totPitchError,prevPitchError);
        if (addedPitch >= 0)
        {
            flSet += addedPitch;
            frSet += addedPitch;
        }
        else
        {
            blSet -= addedPitch;
            brSet -= addedPitch;
        }

        roll = getRoll();
        int addedRoll = pid(pkRoll,ikRoll,dkRoll,desRoll,roll,totRollError,prevRollError);
        if (addedRoll >= 0)
        {
            frSet += addedRoll;
            brSet += addedRoll;
        }
        else
        {
            flSet -= addedRoll;
            blSet -= addedRoll;
        }

        height = getHeight();
        int addedHeight = pid(pkHeight,ikHeight,dkHeight,desHeight,height,totHeightError,prevHeightError);
        flSet += addedHeight;
        frSet += addedHeight;
        blSet += addedHeight;
        brSet += addedHeight;

        if (frSet < 0)
        {
            setFR(0);
        }
        else if (frSet > 255)
        {
            setFR(255);
        }
        else
        {
            setFR(frSet);
        }

        if (flSet < 0)
        {
            setFL(0);
        }
        else if (flSet > 255)
        {
            setFL(255);
        }
        else
        {
            setFL(flSet);
        }

        if (brSet < 0)
        {
            setBR(0);
        }

        else if (brSet > 255)
        {
            setBR(255);
        }

        else
        {
            setBR(brSet);
        }

        if (blSet < 0)
        {
            setBL(0);
        }
        else if (blSet > 255)
        {
            setBL(255);
        }
        else
        {
            setBL(blSet);
        }

        //get pitch,roll,height every dt seconds
        sleep(dt);
    }
}

int pid(int8_t pk, int8_t ik, int8_t dk, int sp, int pv, int &totError, int &prevError)
{
    int error = sp-pv;
    totError += (error*dt);
    int p = pk*error;
    int i = ik*totError;
    int d = dk*((error-prevError)/dt);
    prevError = error;
    return p + i + d;
}
