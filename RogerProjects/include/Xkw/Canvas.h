#ifndef _XCanvas_h
#define _XCanvas_h

/*
**
** Canvas Widget
**
*/

/* Parameters:

 Name			Class		RepType		Default Value
 ----			-----		-------		-------------
 background		Background	pixel		White
 border			BorderColor	pixel		Black
 borderWidth		BorderWidth	int		1
 callback		Callback	Pointer         NULL
 foreground		Foreground	Pixel		Black
 height			Height		int		512
 mappedWhenManaged	MappedWhenManaged	Boolean	True
 width			Width		int		512
 x			Position	int		0
 y			Position	int		0
 label			Label     	String          NULL

*/

#define XtNcursor	"cursor"

typedef struct _CanvasRec	*CanvasWidget;	/* defined in CanvasP.h */
typedef struct _CanvasClassRec	*CanvasWidgetClass; /* defined in CanvasP.h */

extern WidgetClass		canvasWidgetClass;

#endif /* _XCanvas_h */
