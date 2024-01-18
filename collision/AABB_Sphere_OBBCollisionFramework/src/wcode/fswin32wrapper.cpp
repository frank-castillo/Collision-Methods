#if defined(_WIN32_WINNT) && _WIN32_WINNT<0x0500
#undef _WIN32_WINNT
#endif

#if !defined(_WIN32_WINNT)
#define _WIN32_WINNT 0x500
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <windows.h>
#include <gl/gl.h>
#include <gl/glu.h>

#include "fssimplewindow.h"
#include "fswin32keymap.h"

#pragma comment(lib,"kernel32.lib")
#pragma comment(lib,"user32.lib")
#pragma comment(lib,"gdi32.lib")
#pragma comment(lib,"opengl32.lib")
#pragma comment(lib,"glu32.lib")

static FsWin32KeyMapper fsKeyMapper;


HWND hFsWin32Wnd=NULL;  // Loophole used from fsguidlg_win.cpp
HGLRC hRC;
HPALETTE plt;
HDC wDC;





static LONG WINAPI WindowFunc(HWND wnd,UINT msg,WPARAM wp,LPARAM lp);
static void SetPixelFormat(HDC dc,int useDoubleBuffer);
static HPALETTE YsCreatePalette(HDC dc);
static void InitializeOpenGL(HWND wnd);



// For OpenGL set up >>
static int doubleBuffer=0;
// For OpenGL set up <<


class FsMouseEventLog
{
public:
	int eventType;
	int lb,mb,rb;
	int mx,my;
	unsigned int shift,ctrl;
};


#define NKEYBUF 256
static int keyBuffer[NKEYBUF];
static int nKeyBufUsed=0;
static int charBuffer[NKEYBUF];
static int nCharBufUsed=0;
static int nMosBufUsed=0;
static FsMouseEventLog mosBuffer[NKEYBUF];

static int exposure=0;


#define WINSTYLE WS_OVERLAPPED|WS_CAPTION|WS_VISIBLE|WS_SYSMENU|WS_THICKFRAME|WS_MINIMIZEBOX|WS_MAXIMIZEBOX
#if !defined(_UNICODE) && !defined(UNICODE)
	#define WINCLASS "MainWindowClass"
	#define WINNAME "MainWindow"
#else
	#define WINCLASS L"MainWindowClass"
	#define WINNAME L"MainWindow"
#endif

void FsOpenWindow(int x0,int y0,int wid,int hei,int useDoubleBuffer)
{
	WNDCLASS wc;
	HINSTANCE inst;

	inst=GetModuleHandle(NULL);

	wc.style=CS_OWNDC|CS_BYTEALIGNWINDOW;
	wc.lpfnWndProc=WindowFunc;
	wc.cbClsExtra=0;
	wc.cbWndExtra=0;
	wc.hInstance=(HINSTANCE)inst;
#if !defined(_UNICODE) && !defined(UNICODE)
	wc.hIcon=LoadIcon(inst,"MAINICON");
#else
	wc.hIcon=LoadIcon(inst,L"MAINICON");
#endif
	wc.hCursor=LoadCursor(NULL,IDC_ARROW);
	wc.hbrBackground=(HBRUSH)GetStockObject(WHITE_BRUSH);
	wc.lpszMenuName=NULL;
	wc.lpszClassName=WINCLASS;
	if(RegisterClass(&wc))
	{
		doubleBuffer=useDoubleBuffer;


		RECT rc;
		rc.left  =x0;
		rc.top   =y0;
		rc.right =(unsigned long)(x0+wid-1);
		rc.bottom=(unsigned long)(y0+hei-1);
		AdjustWindowRect(&rc,WINSTYLE,FALSE);
		wid  =rc.right-rc.left+1;
		hei  =rc.bottom-rc.top+1;

		hFsWin32Wnd=CreateWindow(WINCLASS,WINNAME,WINSTYLE,x0,y0,wid,hei,NULL,NULL,inst,NULL);
		if(hFsWin32Wnd!=NULL)
		{
			InitializeOpenGL(hFsWin32Wnd);

			ShowWindow(hFsWin32Wnd,SW_SHOWNORMAL);
			UpdateWindow(hFsWin32Wnd);
		}
		else
		{
			printf("Could not open window.\n");
			exit(1);
		}
	}
}

void FsGetWindowSize(int &wid,int &hei)
{
	RECT rect;
	GetClientRect(hFsWin32Wnd,&rect);
	wid=rect.right;
	hei=rect.bottom;
}

void FsPollDevice(void)
{
	MSG msg;
	while(PeekMessage(&msg,hFsWin32Wnd,0,0,PM_REMOVE))
	{
		TranslateMessage(&msg);
		DispatchMessage(&msg);
	}
}

void FsSleep(int ms)
{
	if(ms>0)
	{
		Sleep(ms);
	}
}

DWORD FsPassedTime(bool update)
{
	static DWORD lastTick = GetTickCount();
	DWORD tick,passed=0;

	//tick=GetTickCount();
	tick = timeGetTime();
//	if(lastTick<tick)
	{
		passed=tick-lastTick;
	}
	//else
	//{
	//	passed=1;
	//}
	if(update)
		lastTick=tick;

	return passed;
}

int FsInkey(void)
{
	if(nKeyBufUsed>0)
	{
		int i,keyCode;
		keyCode=keyBuffer[0];
		nKeyBufUsed--;
		for(i=0; i<nKeyBufUsed; i++)
		{
			keyBuffer[i]=keyBuffer[i+1];
		}
		return keyCode;
	}
	return 0;
}

int FsInkeyChar(void)
{
	if(nCharBufUsed>0)
	{
		int i,asciiCode;
		asciiCode=charBuffer[0];
		nCharBufUsed--;
		for(i=0; i<nCharBufUsed; i++)
		{
			charBuffer[i]=charBuffer[i+1];
		}
		return asciiCode;
	}
	return 0;
}

int FsGetKeyState(int fsKeyCode)
{
	int vk;
	vk=fsKeyMapper.FsKeyToVk(fsKeyCode);
	return (GetKeyState(vk)&0x8000)!=0;
}

int FsCheckWindowExposure(void)
{
	int ret;
	ret=exposure;
	exposure=0;
	return ret;
}

void FsGetMouseState(int &lb,int &mb,int &rb,int &mx,int &my)
{
	POINT cur;
	GetCursorPos(&cur);
	ScreenToClient(hFsWin32Wnd,&cur);

	mx=cur.x;
	my=cur.y;

	lb=((GetKeyState(VK_LBUTTON)&0x8000)!=0 ? 1 : 0);
	mb=((GetKeyState(VK_MBUTTON)&0x8000)!=0 ? 1 : 0);
	rb=((GetKeyState(VK_RBUTTON)&0x8000)!=0 ? 1 : 0);
}

int FsGetMouseEvent(int &lb,int &mb,int &rb,int &mx,int &my)
{
	if(0<nMosBufUsed)
	{
		int eventType=mosBuffer[0].eventType;
		mx=mosBuffer[0].mx;
		my=mosBuffer[0].my;
		lb=mosBuffer[0].lb;
		mb=mosBuffer[0].mb;
		rb=mosBuffer[0].rb;

		int i;
		for(i=0; i<nMosBufUsed-1; i++)
		{
			mosBuffer[i]=mosBuffer[i+1];
		}
		nMosBufUsed--;

		return eventType;
	}
	else
	{
		FsGetMouseState(lb,mb,rb,mx,my);
		return FSMOUSEEVENT_NONE;
	}
}

void FsSwapBuffers(void)
{
	HDC dc;
	glFlush();
	dc=wglGetCurrentDC();
	SwapBuffers(dc);
}

static LONG WINAPI WindowFunc(HWND wnd,UINT msg,WPARAM wp,LPARAM lp)
{
	switch(msg)
	{
	case WM_QUERYNEWPALETTE:
	case WM_PALETTECHANGED:
		if(plt!=NULL)
		{
			SelectPalette(GetDC(wnd),plt,FALSE);
			RealizePalette(GetDC(wnd));
		}
		return DefWindowProc(wnd,msg,wp,lp);
	case WM_CREATE:
		wDC=GetDC(wnd);
		SetPixelFormat(wDC,doubleBuffer);
		hRC=wglCreateContext(wDC);
		wglMakeCurrent(wDC,hRC);
		InitializeOpenGL(wnd);
		break;
	case WM_SIZE:
		wDC=GetDC(wnd);
		wglMakeCurrent(wDC,hRC);
		break;
	case WM_PAINT:
		wDC=GetDC(wnd);
		wglMakeCurrent(wDC,hRC);
		exposure=1;
		return DefWindowProc(wnd,msg,wp,lp);
	case WM_COMMAND:
		break;
	case WM_DESTROY:
		exit(1);
		break;
	case WM_SYSKEYDOWN:
		if((lp & (1<<29))!=0 && // Alt
		  (wp==VK_MENU ||
		   wp==VK_OEM_1 ||
		   wp==VK_OEM_PLUS ||
		   wp==VK_OEM_COMMA ||
		   wp==VK_OEM_MINUS ||
		   wp==VK_OEM_PERIOD ||
		   wp==VK_OEM_2 ||
		   wp==VK_OEM_3 ||
		   wp==VK_OEM_4 ||
		   wp==VK_OEM_5 ||
		   wp==VK_OEM_6 ||
		   wp==VK_OEM_7 ||
		   wp==VK_OEM_8 ||
#ifdef VK_OEM_AX
		   wp==VK_OEM_AX ||
#endif
		   wp==VK_OEM_102 ||
		   wp=='0' ||
		   wp=='1' ||
		   wp=='2' ||
		   wp=='3' ||
		   wp=='4' ||
		   wp=='5' ||
		   wp=='6' ||
		   wp=='7' ||
		   wp=='8' ||
		   wp=='9' ||
		   wp=='A' ||
		   wp=='B' ||
		   wp=='C' ||
		   wp=='D' ||
		   wp=='E' ||
		   wp=='F' ||
		   wp=='G' ||
		   wp=='H' ||
		   wp=='I' ||
		   wp=='J' ||
		   wp=='K' ||
		   wp=='L' ||
		   wp=='M' ||
		   wp=='N' ||
		   wp=='O' ||
		   wp=='P' ||
		   wp=='Q' ||
		   wp=='R' ||
		   wp=='S' ||
		   wp=='T' ||
		   wp=='U' ||
		   wp=='V' ||
		   wp=='W' ||
		   wp=='X' ||
		   wp=='Y' ||
		   wp=='Z' ||
		   wp==VK_ESCAPE ||
		   wp==VK_F1 ||
		   wp==VK_F2 ||
		   wp==VK_F3 ||
		   /* wp==VK_F4 || */
		   wp==VK_F5 ||
		   wp==VK_F6 ||
		   wp==VK_F7 ||
		   wp==VK_F8 ||
		   wp==VK_F9 ||
		   wp==VK_F10 ||
		   wp==VK_F11 ||
		   wp==VK_F12 ||
		   wp==VK_RETURN ||
		   wp==VK_NUMLOCK ||
		   wp==VK_NUMPAD0 ||
		   wp==VK_NUMPAD1 ||
		   wp==VK_NUMPAD2 ||
		   wp==VK_NUMPAD3 ||
		   wp==VK_NUMPAD4 ||
		   wp==VK_NUMPAD5 ||
		   wp==VK_NUMPAD6 ||
		   wp==VK_NUMPAD7 ||
		   wp==VK_NUMPAD8 ||
		   wp==VK_NUMPAD9 ||
		   wp==VK_DECIMAL ||
		   wp==VK_DIVIDE ||
		   wp==VK_MULTIPLY ||
		   wp==VK_SUBTRACT ||
		   wp==VK_ADD))
		{
			int keyCode;
			keyCode=fsKeyMapper.VkToFsKey(wp);
			if(keyCode!=0)
			{
				keyBuffer[nKeyBufUsed++]=keyCode;
			}
			return 0;
		}
		return DefWindowProc(wnd,msg,wp,lp);
	case WM_SYSKEYUP:
		return 0;
	case WM_KEYDOWN:
		if(nKeyBufUsed<NKEYBUF)
		{
			int keyCode;
			keyCode=fsKeyMapper.VkToFsKey(wp);
			if(keyCode!=0)
			{
				keyBuffer[nKeyBufUsed++]=keyCode;
			}
		}
		break;
	case WM_CHAR:
		if(nCharBufUsed<NKEYBUF)
		{
			charBuffer[nCharBufUsed++]=wp;
		}
		break;
	case WM_ERASEBKGND:
		return 1;

	case WM_LBUTTONDOWN:
	case WM_LBUTTONUP:
	case WM_MBUTTONDOWN:
	case WM_MBUTTONUP:
	case WM_RBUTTONDOWN:
	case WM_RBUTTONUP:
	case WM_MOUSEMOVE:
		if(nMosBufUsed<NKEYBUF)
		{
			int eventType;
			switch(msg)
			{
			default:
				eventType=FSMOUSEEVENT_NONE;
				break;
			case WM_LBUTTONDOWN:
				eventType=FSMOUSEEVENT_LBUTTONDOWN;
				break;
			case WM_LBUTTONUP:
				eventType=FSMOUSEEVENT_LBUTTONUP;
				break;
			case WM_MBUTTONDOWN:
				eventType=FSMOUSEEVENT_MBUTTONDOWN;
				break;
			case WM_MBUTTONUP:
				eventType=FSMOUSEEVENT_MBUTTONUP;
				break;
			case WM_RBUTTONDOWN:
				eventType=FSMOUSEEVENT_RBUTTONDOWN;
				break;
			case WM_RBUTTONUP:
				eventType=FSMOUSEEVENT_RBUTTONUP;
				break;
			case WM_MOUSEMOVE:
				eventType=FSMOUSEEVENT_MOVE;
				break;
			}

			int lb=((wp & MK_LBUTTON)!=0);
			int mb=((wp & MK_MBUTTON)!=0);
			int rb=((wp & MK_RBUTTON)!=0);
			int shift=((wp & MK_SHIFT)!=0);
			int ctrl=((wp & MK_CONTROL)!=0);
			int mx=LOWORD(lp);
			int my=HIWORD(lp);

			if(eventType==FSMOUSEEVENT_MOVE &&
			   0<nMosBufUsed &&
			   mosBuffer[nMosBufUsed-1].eventType==FSMOUSEEVENT_MOVE &&
			   mosBuffer[nMosBufUsed-1].lb==lb &&
			   mosBuffer[nMosBufUsed-1].mb==mb &&
			   mosBuffer[nMosBufUsed-1].rb==rb &&
			   mosBuffer[nMosBufUsed-1].shift==shift &&
			   mosBuffer[nMosBufUsed-1].ctrl==ctrl)
			{
				mosBuffer[nMosBufUsed-1].mx=mx;
				mosBuffer[nMosBufUsed-1].my=my;
				break;
			}

			mosBuffer[nMosBufUsed].eventType=eventType;
			mosBuffer[nMosBufUsed].lb=lb;
			mosBuffer[nMosBufUsed].mb=mb;
			mosBuffer[nMosBufUsed].rb=rb;
			mosBuffer[nMosBufUsed].shift=shift;
			mosBuffer[nMosBufUsed].ctrl=ctrl;
			mosBuffer[nMosBufUsed].mx=mx;
			mosBuffer[nMosBufUsed].my=my;
			nMosBufUsed++;
		}
		break;

	default:
		return DefWindowProc(wnd,msg,wp,lp);
	}
	return 1;
}

static void SetPixelFormat(HDC dc,int useDoubleBuffer)
{
	static PIXELFORMATDESCRIPTOR pfd=
	{
		sizeof(PIXELFORMATDESCRIPTOR),
		1,
		PFD_DRAW_TO_WINDOW|PFD_SUPPORT_OPENGL,
		PFD_TYPE_RGBA,
		24,
		0,0,0,0,0,0,
		0,
		0,
		0,
		0,0,0,0,
		32,
		0,
		0,
		PFD_MAIN_PLANE,
		0,
		0,0,0
	};

	if(useDoubleBuffer!=0)
	{
		pfd.dwFlags|=PFD_DOUBLEBUFFER;
	}

	int pfm;
	pfm=ChoosePixelFormat(dc,&pfd);
	if(pfm!=0)
	{
		if(SetPixelFormat(dc,pfm,&pfd)!=FALSE)
		{
			plt=YsCreatePalette(dc);
			SelectPalette(dc,plt,FALSE);
			RealizePalette(dc);
			return;
		}
	}

#if !defined(_UNICODE) && !defined(UNICODE)
	MessageBox(NULL,"Error In YsSetPixelFormat.",NULL,MB_OK);
#else
	MessageBox(NULL,L"Error In YsSetPixelFormat.",NULL,MB_OK);
#endif
	exit(1);
}

static unsigned char YsPalVal(unsigned long n,unsigned bit,unsigned sft)
{
	unsigned long msk;
	n>>=sft;
	msk=(1<<bit)-1;
	n&=msk;
	return (unsigned char)(n*255/msk);
}

/* ? lp=LocalAlloc(LMEM_FIXED,sizeof(LOGPALETTE)+n*sizeof(PALETTEENTRY)); */
/* ? LocalFree(lp); */

static HPALETTE YsCreatePalette(HDC dc)
{
	HPALETTE neo;
    PIXELFORMATDESCRIPTOR pfd;
    LOGPALETTE *lp;
    int n,i;

    n=GetPixelFormat(dc);
    DescribePixelFormat(dc,n,sizeof(PIXELFORMATDESCRIPTOR),&pfd);

    if(pfd.dwFlags & PFD_NEED_PALETTE)
    {
        n=1<<pfd.cColorBits;
        lp=(LOGPALETTE *)malloc(sizeof(LOGPALETTE)+n*sizeof(PALETTEENTRY));
        lp->palVersion=0x300;
        lp->palNumEntries=n;
        for (i=0; i<n; i++)
        {
            lp->palPalEntry[i].peRed  =YsPalVal(i,pfd.cRedBits,pfd.cRedShift);
            lp->palPalEntry[i].peGreen=YsPalVal(i,pfd.cGreenBits,pfd.cGreenShift);
            lp->palPalEntry[i].peBlue =YsPalVal(i,pfd.cBlueBits,pfd.cBlueShift);
            lp->palPalEntry[i].peFlags=0;
        }

        neo=CreatePalette(lp);
		free(lp);
	    return neo;
    }
	return NULL;
}

static void InitializeOpenGL(HWND wnd)
{
	RECT rect;

    GetClientRect(wnd,&rect);

    glClearColor(1.0F,1.0F,1.0F,0.0F);
    glClearDepth(1.0F);
//	glDisable(GL_DEPTH_TEST);

	glViewport(0,0,rect.right,rect.bottom);

    glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0,(float)rect.right-1,(float)rect.bottom-1,0,-1,1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glShadeModel(GL_SMOOTH);
	glPointSize(1);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
	glColor3ub(0,0,0);
}
