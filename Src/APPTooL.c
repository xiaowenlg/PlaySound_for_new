#include "APPTooL.h"
#include <string.h>
#include "BspConfig.h"
uint8_t lasttrivalue[TRLENGHT] = { 0 };


uint8_t TimData[15] = { S_SPORTTIM, S_HOUR, S_MINUTE, S_SECOND };//活动时间
uint8_t KcalData[25] = { S_THISKCAL, S_KCAL, S_WELCOME };//消耗热量+欢迎再次使用
/*
函数:    SingleTrig
函数全名:  SingleTrig
函数类型:    public 
返回值:   void
Qualifier:单次触发函数
参数: fun p   回调函数,触发后的处理函数
参数: int val 输入的值
参数:inval    回调函数用参数
参数: index 触发器索引号
参数: com   比较值
//数组的作用:区分全局变量用
*/
void SingleTrig(TrlCallback p, uint8_t val, uint8_t inval, uint8_t index,uint8_t com)
{
	if (index >= 0 && index < TRLENGHT)
	{
		index = index;
	}
	else if (index >= 10)
	{
		index = 9;
	}
	else
	{
		index = 0;
	}
	if (val != lasttrivalue[index])
	{
		if (val == com)  //低电平触发
		{
			(*p)(inval); //
		}
		osDelay(10);
	}
	lasttrivalue[index] = val;
}
uint8_t DataSeparation(double dat, uint8_t *res_dat)//数据分离-------------带小数点
{
	uint8_t intlen = 0, flen = 0, reslen;
	uint16_t intData = (uint16_t)dat;//取整数部分
	uint8_t pointarray[4] = { 0 };
	double fData = dat - intData;//取小数部分o
	//printf("fData:%f\r\n", fData);
	pointarray[0] = S_POINT;
	//获取十分位部分
	uint8_t sf = (int)(fData * 10) % 10;
	//获取百分位部分
	uint8_t bf = (int)(fData * 100) % 10;
	//获取千分位部分
	uint8_t qf = (int)(dat * 1000) % 10;
	//printf("qf:%d\r\n", (int)(dat * 1000));
	//判断千分位是否大于等于5

	intlen = PrepartData(intData, res_dat);
	if (fData == 0)
	{
		reslen = intlen;
	}
	else
	{
		if (bf == 0 && qf == 0)
		{
			//insertOneByte(playarray,intlen+2,PLAY_POINT,intlen)

			pointarray[1] = sf;
			reslen = insertArray(res_dat, intlen, pointarray, 2, intlen);
		}
		else if (bf != 0 && qf == 0)
		{
			pointarray[1] = sf;
			pointarray[2] = bf;
			reslen = insertArray(res_dat, intlen, pointarray, 3, intlen);
		}
		else
		{
			pointarray[1] = sf;
			pointarray[2] = bf;
			pointarray[3] = qf;
			reslen = insertArray(res_dat, intlen, pointarray, 4, intlen);
		}
	}
	return reslen;
}
/*
uint8_t PrepartData(uint16_t dat, uint8_t *res_dat)// 语音地址分解计算
{
	uint8_t len, q, b, s, g;
	if (dat > 9999)
		dat = 9999;
	q = dat / 1000; //千位
	b = dat % 1000 / 100;//百位
	s = dat % 100 / 10; //十位
	g = dat % 10;  //个位

	if ((dat <= 9) && (dat >= 0)) //0-9
	{
		res_dat[0] = dat;//语音地址5-14
		len = 1;
	}
	if ((dat > 9) && (dat <= 99))//10-99
	{
		if ((dat % 10) == 0)
		{
			res_dat[0] = s;
			res_dat[1] = 10;//语音地址15 是十
			len = 2;
		}
		else
		{
			res_dat[0] = s;
			res_dat[1] = 10;
			res_dat[2] = g;
			len = 3;
		}
	}
	if ((dat > 99) && (dat <= 999))//100-999
	{
		if ((dat % 100) == 0)
		{
			res_dat[0] = b;
			res_dat[1] = 11;//语音地址16 是百
			len = 2;
		}
		else if ((dat % 10) == 0)
		{
			res_dat[0] = b;
			res_dat[1] = 11;//语音地址16 是百
			res_dat[2] = s;
			res_dat[3] = 10;//语音地址15 是十
			len = 4;
		}
		else if (dat % 100 > 0 && dat % 100 < 10)
		{
			res_dat[0] = b;
			res_dat[1] = 11;//语音地址16 是百
			res_dat[2] = 0;
			res_dat[3] = g;
			len = 4;

		}
		else
		{
			res_dat[0] = b;
			res_dat[1] = 11;//语音地址16 是百
			res_dat[2] = s;
			res_dat[3] = 10;//语音地址15 是十
			res_dat[4] = g;
			len = 5;
		}

	}
	if ((dat > 999) && (dat <= 9999))//1000-9999
	{
		if ((dat % 1000) == 0)
		{
			res_dat[0] = q;
			res_dat[1] = 12;//语音地址17 是千位
			len = 2;
		}
		else if (b == 0 && s == 0 && g != 0)//001
		{
			res_dat[0] = q;
			res_dat[1] = 12;//语音地址16 是千位
			res_dat[2] = 0;
			res_dat[3] = g;
			len = 4;
		}
		else if (b == 0 && s != 0 && g == 0)//010
		{
			res_dat[0] = q;
			res_dat[1] = 12;//语音地址16 是千位
			res_dat[2] = 0;
			res_dat[3] = s;//语音地址15 是十
			res_dat[4] = 10;
			len = 5;
		}
		else if (b == 0 && s != 0 && g != 0)//011--------百位=0 十位和个位不等于0----------------因为此处少判断一种状态，导致重复播报
		{
			res_dat[0] = q;
			res_dat[1] = 12;
			res_dat[2] = 0;
			res_dat[3] = s;//语音地址15 是十
			res_dat[4] = 10;
			res_dat[5] = g;
			len = 6;

		}
		else if (b != 0 && s == 0 && g == 0)//100
		{
			res_dat[0] = q;
			res_dat[1] = 12;//语音地址16 是千位
			res_dat[2] = b;//语音地址15 是十
			res_dat[3] = 11;
			len = 4;
		}
		else if (b != 0 && s != 0 && g == 0)//110*********************************
		{
			res_dat[0] = q;
			res_dat[1] = 12;//语音地址16 是千位
			res_dat[2] = b;//语音地址15 是十
			res_dat[3] = 11;
			res_dat[4] = s;
			res_dat[5] = 10;
			len = 6;
		}
		else if (b != 0 && s == 0 && g != 0)//101
		{
			res_dat[0] = q;
			res_dat[1] = 12;//语音地址16 是千位
			res_dat[2] = b;//语音地址15 是十
			res_dat[3] = 11;
			res_dat[4] = 0;
			res_dat[5] = g;
			len = 6;
		}
		//else if(b!=0&&s!=0&&g!=0)//111
		else//111
		{
			res_dat[0] = q;
			res_dat[1] = 12;//语音地址16 是千位
			res_dat[2] = b;
			res_dat[3] = 11;
			res_dat[4] = s;
			res_dat[5] = 10;
			res_dat[6] = g;
			len = 7;
		}


	}

	return len;//返回数组长度
}*/
uint8_t PrepartData(uint16_t dat, uint8_t *res_dat)// 语音地址分解计算
{
	uint8_t len, q, b, s, g;
	if (dat > 9999)
		dat = 9999;
	q = dat / 1000; //千位
	b = dat % 1000 / 100;//百位
	s = dat % 100 / 10; //十位
	g = dat % 10;  //个位

	if ((dat <= 9) && (dat >= 0)) //0-9
	{
		res_dat[0] = dat + 5;//语音地址5-14
		len = 1;
	}
	if ((dat > 9) && (dat <= 99))//10-99
	{
		if ((dat % 10) == 0)
		{
			res_dat[0] = s + 5;
			res_dat[1] = 15;//语音地址15 是十
			len = 2;
		}
		else
		{
			res_dat[0] = s + 5;
			res_dat[1] = 15;
			res_dat[2] = g + 5;
			len = 3;
		}
	}
	if ((dat > 99) && (dat <= 999))//100-999
	{
		if ((dat % 100) == 0)
		{
			res_dat[0] = b + 5;
			res_dat[1] = 16;//语音地址16 是百
			len = 2;
		}
		else if ((dat % 10) == 0)
		{
			res_dat[0] = b + 5;
			res_dat[1] = 16;//语音地址16 是百
			res_dat[2] = s + 5;
			res_dat[3] = 15;//语音地址15 是十
			len = 4;
		}
		else if (dat % 100 > 0 && dat % 100 < 10)
		{
			res_dat[0] = b + 5;
			res_dat[1] = 16;//语音地址16 是百
			res_dat[2] = 5;
			res_dat[3] = g + 5;
			len = 4;

		}
		else
		{
			res_dat[0] = b + 5;
			res_dat[1] = 16;//语音地址16 是百
			res_dat[2] = s + 5;
			res_dat[3] = 15;//语音地址15 是十
			res_dat[4] = g + 5;
			len = 5;
		}

	}
	if ((dat > 999) && (dat <= 9999))//1000-9999
	{
		if ((dat % 1000) == 0)
		{
			res_dat[0] = q + 5;
			res_dat[1] = 17;//语音地址17 是千位
			len = 2;
		}
		else if (b == 0 && s == 0 && g != 0)//001
		{
			res_dat[0] = q + 5;
			res_dat[1] = 17;//语音地址16 是千位
			res_dat[2] = 5;
			res_dat[3] = g + 5;
			len = 4;
		}
		else if (b == 0 && s != 0 && g == 0)//010
		{
			res_dat[0] = q + 5;
			res_dat[1] = 17;//语音地址16 是千位
			res_dat[2] = 5;
			res_dat[3] = s + 5;//语音地址15 是十
			res_dat[4] = 15;
			len = 5;
		}
		else if (b == 0 && s != 0 && g != 0)//011--------百位=0 十位和个位不等于0----------------因为此处少判断一种状态，导致重复播报
		{
			res_dat[0] = q + 5;
			res_dat[1] = 17;
			res_dat[2] = 5;
			res_dat[3] = s + 5;//语音地址15 是十
			res_dat[4] = 15;
			res_dat[5] = g + 5;
			len = 6;

		}
		else if (b != 0 && s == 0 && g == 0)//100
		{
			res_dat[0] = q + 5;
			res_dat[1] = 17;//语音地址16 是千位
			res_dat[2] = b + 5;//语音地址15 是十
			res_dat[3] = 16;
			len = 4;
		}
		else if (b != 0 && s != 0 && g == 0)//110*********************************
		{
			res_dat[0] = q + 5;
			res_dat[1] = 17;//语音地址16 是千位
			res_dat[2] = b + 5;//语音地址15 是十
			res_dat[3] = 16;
			res_dat[4] = s + 5;
			res_dat[5] = 15;
			len = 6;
		}
		else if (b != 0 && s == 0 && g != 0)//101
		{
			res_dat[0] = q + 5;
			res_dat[1] = 17;//语音地址16 是千位
			res_dat[2] = b + 5;//语音地址15 是十
			res_dat[3] = 16;
			res_dat[4] = 5;
			res_dat[5] = g + 5;
			len = 6;
		}
		//else if(b!=0&&s!=0&&g!=0)//111
		else//111
		{
			res_dat[0] = q + 5;
			res_dat[1] = 17;//语音地址16 是千位
			res_dat[2] = b + 5;
			res_dat[3] = 16;
			res_dat[4] = s + 5;
			res_dat[5] = 15;
			res_dat[6] = g + 5;
			len = 7;
		}


	}

	return len;//返回数组长度
}
//************************************
// 函数:    MergedData
// 函数全名:  MergedData
// 函数类型:    public 
// 返回值:   uint8_t
// Qualifier: 输入活动次数返回数组长度和数组
// 参数: double numdata //1,活动次数2,消耗热量
// 参数: uint8_t * dat  //整理好的数组
//************************************
uint8_t MergedData(double numdata, uint8_t *dat, uint8_t com)//dat最小容量9
{
	uint8_t reslen = 0, templen = 0;

	switch (com)
	{
	case 1:
		dat[0] = S_THISSPORT; //活动次数数组
		dat[1] = S_NUMBER;
		uint8_t tempdat[10] = { 0 };
		templen = DataSeparation(numdata, tempdat);
		reslen = insertArray(dat, 2, tempdat, templen, 1);
		break;
	case 2:
		dat[0] = S_THISKCAL; //消耗热量数组
		dat[1] = S_KCAL;
		dat[2] = S_WELCOME;
		uint8_t tdat[16] = { 0 };
		templen = DataSeparation(numdata, tdat);
		reslen = insertArray(dat, 3, tdat, templen, 1);
	default:
		break;
	}
	return reslen;
}
uint8_t GetTimData(uint32_t tdata, uint8_t *dat)//dat最小容量16
{
	uint8_t reslen = 0;
	dat[0] = S_SPORTTIM, dat[1] = S_HOUR, dat[2] = S_MINUTE; dat[3] = S_SECOND;
	uint8_t th = tdata / 3600, tme = tdata % 3600 / 60, ts = tdata % 3600 % 60;//时，分，秒
	uint8_t thlen = 0, tmelen = 0, tslen = 0;
	uint8_t thdata[8] = { 0 }, tmedata[4] = { 0 }, tsdata[4] = { 0 }; //时，分，秒的数组
	thlen = PrepartData(th, thdata);//时
	tmelen = PrepartData(tme, tmedata);//分
	tslen = PrepartData(ts, tsdata);//秒

	reslen = insertArray(dat, 4, thdata, thlen, 1);
	reslen = insertArray(dat, reslen, tmedata, tmelen, reslen - 2);
	reslen = insertArray(dat, reslen, tsdata, tslen, reslen - 1);
	return reslen;

}
uint8_t GetPlayData(uint16_t num, uint16_t tim, double mcal,uint8_t *dat)//此处dat数组容量最小38
{
	uint8_t reslen = 0;
	uint8_t nlen = 0, tlen = 0, clen = 0;
	uint8_t tdat[20] = {0}, cdat[10] = {0};
	nlen = MergedData(num, dat, 1);
	tlen = GetTimData(tim, tdat);
	clen = MergedData(mcal,cdat,2);
	reslen = insertArray(dat, nlen, tdat, tlen, nlen);  //插入时间
	reslen = insertArray(dat, reslen, cdat, clen, reslen);//插入热量
	return reslen;
}
//数组操作
//************************************
// 函数:    insertArray
// 函数全名:  insertArray
// 函数类型:    public 
// 返回值:   uint8_t         数组长度   -1 失败
// Qualifier: 在一个数组中插入另一个数组
// 参数: uint8_t * arr    原始数组
// 参数: uint16_t arrlen  原始数组长度（内部有效数据长度）
// 参数: uint8_t * inarr  插入数组
// 参数: uint16_t inarrlen 插入数组长度
// 参数: uint16_t pos      插入位置 从0开始到最大65535
//************************************
int insertArray(uint8_t *arr, uint16_t arrlen, uint8_t *inarr, uint16_t inarrlen,uint16_t pos)
{
	int res = -2;
	uint16_t i = 0;
	if ((arrlen + inarrlen) < 65536)
	{
		//memcpy(&buf_date[bufdateIndex], resdate, resdatelen);
		memcpy(&arr[pos + inarrlen], &arr[pos], arrlen - pos + 1);//数组从pos后，后移inarrlen位
		memcpy(&arr[pos], inarr, inarrlen);

		res = arrlen + inarrlen;
	}
	else
		res = -1;
	return res;
}
//插入一个字符
int insertOneByte(uint8_t *arr, uint16_t arrlen, uint8_t indata, uint16_t pos)
{
	int res = -2;
	int i = 0;
	if ((arrlen + 1) < 65536)
	{
		for (i = arrlen; i >= pos; i--)
		{
			arr[i + 1] = arr[i];
		}
		arr[pos] = indata;
		res = arrlen + 1;
	}
	else
		res = -1;
	return res;
}
//字符串拼接函数(二维码信息拼接)
//************************************
// 函数:    StrJoin
// 函数全名:  StrJoin
// 函数类型:    public 
// 返回值:   uint8_t
// Qualifier:
// 参数: char * mac       蓝牙地址
// 参数: char * id		  器材ID号
// 参数: char * lastdata  最后的数组
//************************************
uint8_t StrJoin(char *mac, char*id,char *lastdata)
{

	//strcpy(lastdata, QR_HEAD);//头部复制进数组
	//strcat(lastdata, mac);
	//strcat(lastdata, QR_EQNUM);
	//strcat(lastdata, id);
	//strcat(lastdata, QR_END);
	//return (strlen(QR_HEAD)+strlen(mac)+strlen(QR_EQNUM)+strlen(id)+strlen(QR_END)+1);
	return 0;
}

//卡路里计算
uint16_t ConsumeHeat(float weight, float tim, float v)
{
	return (uint16_t)((3 * weight * tim * v * 10) / 40);
}