MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x00000000, LENGTH = 1024K
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K

  /* Uncomment these instead when using a SoftDevice (e.g. S140 v7.3.0): */
  /*
  FLASH : ORIGIN = 0x00027000, LENGTH = 868K
  RAM   : ORIGIN = 0x20020000, LENGTH = 128K
  */
}
