
#pragma pack(push,1)
#define UCHAR 	uint8_t
#define USHORT 	uint16_t
#define UINT 	uint32_t

typedef enum _IO_REQUEST_TYPE { 
	IO_TYPE_UNKNOWN,
    IO_TYPE_URB,
	IO_TYPE_CLAIMDEVICE_OBSOLETE,
	IO_TYPE_ABORT_PIPE,
	IO_TYPE_RESET_PIPE,
	IO_TYPE_RESET_PORT,
    IO_TYPE_INTERNAL_URB,
    IO_TYPE_UNLINK_URB,
    IO_TYPE_GET_VERSION,
    IO_TYPE_CLAIMDEVICE,
    IO_TYPE_SET_CONFIGURATION,
    IO_TYPE_CLAIMDEVICE2,
    IO_TYPE_CLAIMDEVICE3,
    IO_TYPE_GET_VERSION2,
} IO_REQUEST_TYPE;
    
typedef enum _DEVICE_PNP_STATE {
    PNP_NotStarted = 0,         // Not started yet
    PNP_Started,                // Device has received the START_DEVICE IRP
    PNP_StopPending,            // Device has received the QUERY_STOP IRP
    PNP_Stopped,                // Device has received the STOP_DEVICE IRP
    PNP_RemovePending,          // Device has received the QUERY_REMOVE IRP
    PNP_SurpriseRemovePending,  // Device has received the SURPRISE_REMOVE IRP
    
    PNP_Deleted,                // Device has received the REMOVE_DEVICE IRP (kein PDO mehr vorhaden)
    PNP_Unknown                 // Unknown state                             (kein PDO mehr vorhaden)
} DEVICE_PNP_STATE;

typedef struct _VUSB_CTRL_TRANSFER {
    UCHAR  bmRequestType;
    UCHAR  bRequest;
    USHORT wValue;
    USHORT wIndex;
    USHORT wLength;
} VUSB_CTRL_TRANSFER, *PVUSB_CTRL_TRANSFER;


struct _VUSB_URB_HEADER {
    //! USB Adresse
    UCHAR addr;
    //! Busnummer
    UCHAR busnr;
    //! Endpoint
    UCHAR endpoint;
    //! Type
    UCHAR type;
    //! Fehlercode
    //DWORD error;
    //! Maximale Datenlaenge
    UINT maxLength;
    //! Laenge der Daten
    UINT length;
};

typedef struct _VUSB_URB_BULK_OR_INTERRUPT_REQUEST {
    struct _VUSB_URB_HEADER     hdr;
    UCHAR                       data[1];
} VUSB_URB_BULK_OR_INTERRUPT_REQUEST, *PVUSB_URB_BULK_OR_INTERRUPT_REQUEST;

typedef struct _VUSB_URB_CTRL_REQUEST {
    struct _VUSB_URB_HEADER     hdr;
    struct _VUSB_CTRL_TRANSFER  ctrl;
    UCHAR                       data[1];
} VUSB_URB_CTRL_REQUEST, *PVUSB_URB_CTRL_REQUEST;
    
typedef struct _VUSB_DATA {
    union {
        struct _VUSB_URB_HEADER                     header;
        struct _VUSB_URB_CTRL_REQUEST               urbCtrlRequest;
        struct _VUSB_URB_BULK_OR_INTERRUPT_REQUEST  urbBulkOrInterruptRequest;
        };
    } VUSB_DATA, *PVUSB_DATA;

// awful duplicate to ../inc/driver.h ?!
typedef struct _VUSB_GET_URB {
    UINT       size;                     
    UINT       descriptorType;
    UCHAR      busno;
    UCHAR      addr;
    UINT       ipl;
    UINT       pIoRequestHandle;   // Zeiger auf den IoRequest im Treiber
    UCHAR      dataType;
    UINT       dataLength;
    UINT       dataCntr;
    USHORT     useSSL;
    VUSB_DATA  data;
    UCHAR      buffer[1];          // [0] is smarter, but not windows compatible
} VUSB_GET_URB, *PVUSB_GET_URB;

// awful duplicate to ../inc/driver.h ?!
typedef struct _VUSB_SET_URB{
    UINT    size;                     
    UINT    descriptorType;
    UCHAR   busno;
    UCHAR   addr;
    UINT    ipl;
    UINT    pIoRequestHandle;
    UINT    status;
    UINT    error;
    UCHAR   dataType;
    UINT    dataLength;
    UINT    dataCntr;
    UCHAR   data[1];               // [0] is smarter, but not windows compatible
} VUSB_SET_URB, *PVUSB_SET_URB;

#pragma pack(pop)



