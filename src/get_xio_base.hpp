// CREDIT FOR THIS FUNCTION DUE TO HOWIE KATZ OF NTC AND STEVE FORD
// THIS WILL FIND THE PROPER XIO BASE SYSFS NUMBER
// PORTED TO C FORM HOWIE'S PYTHON CODE WITH THE HELP OF STEVE:
// https://gist.github.com/howientc/606545e0ff47e2cda61f14fca5c46eee
// HAT TIP TO:
// http://stackoverflow.com/questions/8149569/scan-a-directory-to-find-files-in-c
// http://stackoverflow.com/questions/612097/how-can-i-get-the-list-of-files-in-a-directory-using-c-or-c
#define GPIO_PATH "/sys/class/gpio"
#define EXPANDER "pcf8574a\n"

/* returns -1 for error */
int get_xio_base(void)
{
  char label_file[FILENAME_BUFFER_SIZE];
  FILE *label_fp;
  char base_file[FILENAME_BUFFER_SIZE];
  FILE *base_fp;
  // Makes use of static variable xio_base_address to maintain state between
  // calls.  First time this is called, xio_base_address is -1, so the actual
  // base address is calculated and stored in xio_base_address.  Subsequent
  // calls just use the previously-calculated value.
  static int xio_base_address = -1;

  DIR *dir;
  struct dirent *ent;
  struct stat sbuf;

  if (xio_base_address != -1)
    return xio_base_address;

  dir = opendir (GPIO_PATH);
  if (dir == NULL) {
    char err[256];
    snprintf(err, sizeof(err), "get_xio_base: could not open '%s' (%s)", GPIO_PATH, strerror(errno));
    add_error_msg(err);
    return -1;
  }

  while (xio_base_address == -1 && (ent = readdir (dir)) != NULL) {
    lstat(ent->d_name,&sbuf);
    if (S_ISDIR(sbuf.st_mode)) {
      if (strcmp(".",ent->d_name) == 0 || strcmp("..",ent->d_name) == 0) {
        continue;  /* skip "." and ".." entries */
      }

      snprintf(label_file, sizeof(label_file), "%s/%s/label", GPIO_PATH, ent->d_name); BUF2SMALL(label_file);
      label_fp = fopen(label_file, "r");
      if (label_fp != NULL) {
        char input_line[80];  input_line[0] = '\0';
        char *s = fgets(input_line, sizeof(input_line), label_fp); BUF2SMALL(input_line);
        fclose(label_fp);
        if (s != NULL && strcmp(input_line, EXPANDER) == 0) {
          /* Found the expander, get the contents of base */
          snprintf(base_file, sizeof(base_file), "%s/%s/base", GPIO_PATH, ent->d_name); BUF2SMALL(base_file);
          base_fp = fopen(base_file, "r");
          if (base_fp == NULL) {
            char err[256];
            snprintf(err, sizeof(err), "get_xio_base: could not open '%s' (%s)", base_file, strerror(errno));
            add_error_msg(err);
            break;  /* error, exit loop */
          }
          s = fgets(input_line, sizeof(input_line), base_fp); BUF2SMALL(input_line);
          fclose(base_fp);
          if (s == NULL) {
            char err[256];
            snprintf(err, sizeof(err), "get_xio_base: could not read '%s' (%s)", base_file, strerror(errno));
            add_error_msg(err);
            break;  /* error, exit loop */
          }
          /* Remember the value in the static local. */
          xio_base_address = atoi(input_line);
        }
      }  /* if label file is open */
    }  /* if isdir */
  }  /* while */
  closedir (dir);

  return xio_base_address; 
} /* get_xio_base */
