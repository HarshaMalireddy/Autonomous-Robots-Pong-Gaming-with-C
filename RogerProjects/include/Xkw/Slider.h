#ifndef _Slider_h
#define _Slider_h

/*
**
** Slider Widget
**
*/

#include <X11/Xmu/Converters.h>

/* Parameters:

 Name			Class		RepType		Default Value
 ----			-----		-------		-------------
 background		Background	Pixel		White
 border			BorderColor	Pixel		Black
 borderWidth		BorderWidth	Dimension	1
 destroyCallback	Callback	Function	NULL
 foreground		Color		Pixel		Black
 foregroundThumb	Color		Pixel		Black
 height			Height		Dimension	length or thickness
 valueProc		Callback	Function	NULL
 length			Length		Dimension	1
 mappedWhenManaged	MappedWhenManaged	Boolean	True
 orientation		Orientation	XtOrientation	XtorientVertical
 reverseVideo		ReverseVideo	Boolean		False
 scrollDCursor		Cursor		Cursor		XC_sb_down_arrow
 scrollHCursor		Cursor		Cursor		XC_sb_h_double_arrow
 scrollLCursor		Cursor		Cursor		XC_sb_left_arrow
 scrollRCursor		Cursor		Cursor		XC_sb_right_arrow
 scrollUCursor		Cursor		Cursor		XC_sb_up_arrow
 scrollVCursor		Cursor		Cursor		XC_sb_v_double_arrow
 sensitive		Sensitive	Boolean		True
 thickness		Thickness	Dimension	15
 thumb			Thumb		Pixmap		Grey
 width			Width		Dimension	thickness or length
 label			Label		String          NULL
 value			Value		Double		0.0
 shown			Value		Double		0.0
 startOfThumb		Value		Double		0.0
 min			Value		Double		-1.0
 max			Value		Double		1.0
 notifyCont		Notify		Boolean		False
 intSlider		Notify		Boolean		False
 attachThumb		AttachThumb	Position	Self

*/

/* 
** Most things we need are in StringDefs.h 
*/

#define XtCMinimumThumb	"MinimumThumb"
#ifndef XtCValue
#define XtCValue	"Value"
#endif
#ifndef XtCIntValue
#define XtCIntValue	"IntValue"
#endif
#ifndef XtCLabel
#define XtCLabel	"Label"
#endif
#ifndef XtCNotify
#define XtCNotify	"Notify"
#endif
#define XtCAttachThumb	"AttachThumb"

#define XtRDouble	"Double"
#define Double		double
#define Min		2
#define Max		1
#define Self		0

#define XtNvalueProc	"valueProc"
#define XtNminimumThumb	"minimumThumb"
#define XtNstartOfThumb	"startOfThumb"
#define XtNforegroundThumb	"foregroundThumb"
#ifndef XtNlabel
#define XtNlabel	"label"
#endif
#define XtNnotifyMid	"notifyMid"
#define XtNnotifyCont	"notifyCont"
#define XtNintSlider	"intSlider"
#ifndef XtNintValue
#define XtNintValue	"intValue"
#endif
#ifndef XtNvalue
#define XtNvalue	"value"
#endif
#ifndef XtNmin
#define XtNmin		"min"
#endif
#ifndef XtNmax
#define XtNmax		"max"
#endif
#define XtNattachThumb	"attachThumb"
#define XtNinc	"inc"

typedef struct _SliderRec	*SliderWidget;
typedef struct _SliderClassRec	*SliderWidgetClass;

extern WidgetClass	sliderWidgetClass;

extern void		XawSliderSetValue(); /* Widget slider, double value */
extern void		XawIntSliderSetValue(); /* Widget slider, int value */

#endif /* _Slider_h */
