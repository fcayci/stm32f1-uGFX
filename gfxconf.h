#ifndef _GFXCONF_H
#define _GFXCONF_H

// No OS
#define GFX_USE_OS_RAW32         TRUE

// Use Display

/* Features for the GDISP sub-system. */
#define GFX_USE_GDISP            TRUE
#define GDISP_STARTUP_COLOR      HTML2COLOR(0xC0C0C0)
#define GDISP_NEED_VALIDATION    TRUE
#define GDISP_NEED_CLIP          TRUE
#define GDISP_NEED_TEXT          TRUE
#define GDISP_NEED_CIRCLE        TRUE
#define GDISP_INCLUDE_FONT_UI2   TRUE
#define GDISP_NEED_IMAGE         TRUE

/* GDISP image decoders */
#define GDISP_NEED_IMAGE_BMP     TRUE
//#define GDISP_NEED_IMAGE_PNG    TRUE
//#define GDISP_NEED_IMAGE_GIF    TRUE
//#define GDISP_NEED_IMAGE_NATIVE TRUE

/**
 * The image file must be stored on a GFILE file-system.
 * Use either GFILE_NEED_NATIVEFS or GFILE_NEED_ROMFS (or both).
 *
 * The ROMFS uses the file "romfs_files.h" to describe the set of files in the ROMFS.
 */
#define GFX_USE_GFILE            TRUE
#define GFILE_NEED_ROMFS         TRUE
//#define GFILE_NEED_NATIVEFS      TRUE

#endif /* _GFXCONF_H */
