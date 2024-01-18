#include <windows.h>
#include <stdio.h>
#include <ctype.h>

static char *YsStrtok(char **src);
static int YsArguments(int *ac,char *av[],int mxac,char *src);

extern int main(int ac,char *av[]);

int PASCAL WinMain(HINSTANCE inst,HINSTANCE dumb,LPSTR param,int show)
{
	int ac;
	static char *av[512],tmp[4096],prog[MAX_PATH];

	strcpy(prog,"Unknown");
	GetModuleFileNameA(inst,prog,260);

	strncpy(tmp,param,256);
	av[0]=prog;

	YsArguments(&ac,av+1,510,tmp);

	return main(ac+1,av);
}




static int YsArguments(int *ac,char *av[],int mxac,char *src)
{
	char *arg;
	int len;

	while(*src==' ' || *src=='\t')
	{
		src++;
	}

	len=strlen(src)-1;
	while(len>=0 && isprint(src[len])==0)
	{
		src[len]=0;
		len--;
	}

	*ac=0;
	while((arg=YsStrtok(&src))!=NULL && *ac<mxac)
	{
		av[*ac]=arg;
		(*ac)++;
	}

	if(*ac<mxac)  // 2007/06/22 Added this check.
	{
		av[*ac]=NULL;
	}

	return *ac;
}

static char *YsStrtok(char **src)
{
	char *r;
	switch(**src)
	{
	case 0:
		return NULL;
	case '\"':
		(*src)++;
		r=(*src);
		while((**src)!='\"' && (**src)!=0)
		{
			(*src)++;
		}
		break;
	default:
		r=(*src);
		while((**src)!=' ' && (**src)!='\t' && (**src)!=0)
		{
			(*src)++;
		}
		break;
	}
	if((**src)!=0)
	{
		(**src)=0;
		(*src)++;
	}
	while((**src)!=0 && ((**src)==' ' || (**src)=='\t'))
	{
		(*src)++;
	}
	return r;
}
