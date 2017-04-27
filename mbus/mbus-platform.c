#if defined(_MSC_VER)

#include <Windows.h>
#include <stdio.h>

BOOL WINAPI DllMain(
    HINSTANCE hinstDLL,  // handle to DLL module
    DWORD fdwReason,     // reason for calling function
    LPVOID lpReserved )  // reserved
{
	int iResult;
	WSADATA wsaData;

    // Perform actions based on the reason for calling.
    switch( fdwReason ) 
    { 
        case DLL_PROCESS_ATTACH:
            // Initialize once for each new process.
            // Return FALSE to fail DLL load.

			// Initialize libmbus
			iResult = mbus_init();
			if (iResult != 0)
			{
				fprintf(stderr, "mbus_init() failed: %d\n", iResult);
				return FALSE;
			}

			// Initialize Winsock
			iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
			if (iResult != 0)
			{
				fprintf(stderr, "WSAStartup failed: %d\n", iResult);
				return FALSE;
			}
            break;

        case DLL_THREAD_ATTACH:
         // Do thread-specific initialization.
            break;

        case DLL_THREAD_DETACH:
         // Do thread-specific cleanup.
            break;

        case DLL_PROCESS_DETACH:
         // Perform any necessary cleanup.
			WSACleanup();
            break;
    }

    return TRUE;  // Successful DLL_PROCESS_ATTACH.
}
#else

#endif
