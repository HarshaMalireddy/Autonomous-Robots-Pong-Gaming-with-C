/*
**
**      xkw.h:	
**      author:		Kamal Souccar
**      date:		Mar 26, 1991
**
*/

#define ScrollX (LASTEvent+1)
#define ScrollY (LASTEvent+2)

#include <X11/Intrinsic.h>
#include <X11/StringDefs.h>
#include <X11/cursorfont.h>
#include <X11/Xatom.h>
#include <X11/Shell.h>
#include <X11/Xaw/Paned.h>
#include <X11/Xaw/Simple.h>
#include <X11/Xaw/Label.h>
#include <X11/Xaw/Command.h>
#include <X11/Xaw/Cardinals.h>
#include <X11/Xaw/Form.h>
#include <X11/Xaw/Dialog.h>
#include <X11/Xaw/Toggle.h>
#include <X11/Xaw/List.h>
#include "Canvas.h"
#include "Slider.h"

#define XkwMakeToplevel		XtAppInitialize

extern Widget	XkwMakePane();
extern Widget	XkwMakeForm();
extern Widget	XkwMakeLabel();
extern Widget	XkwMakeCommand();
extern Widget	XkwMakeCommandBR();
extern Widget	XkwMakeCommandR();
extern Widget	XkwMakeToggle();
extern Widget	XkwMakeList();
extern Widget	XkwMakeSlider();
extern Widget	XkwMakeIntSlider();
extern Widget	XkwMakeSafeSlider();
extern Widget	XkwMakeCanvas();
extern Widget	XkwMakePopup();
extern Widget	XkwMakeDialog();
extern double	XkwGetSliderValue();
extern void	XkwSetSliderValue();
extern void	XkwSetSliderSafe();
extern void	XkwSetWidgetLabel();
extern void	XkwSetWidgetColor();
extern void	XkwGetWidgetColor();
extern void     XkwSetWidgetFont();
extern void     XkwSetWidgetBorderColor();
extern void	XkwWidgetHighlight();
extern void	XkwWidgetUnhighlight();
extern Widget   XkwMakeMenuButton();
extern Widget   XkwMakeSimpleMenu();
extern Widget   XkwMakeSmeBSB();
extern Widget   XkwMakeScrollCanvas();
extern int      XkwGetSliderOrient();
extern unsigned long XkwParseColor();
extern Widget   XkwMakeShell();
extern void     XkwGetWidgetDim();

