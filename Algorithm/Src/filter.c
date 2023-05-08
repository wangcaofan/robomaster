//
// Created by Yuanbin on 22-10-3.
//
#include "filter.h"

/**
 *@brief	���׵�ͨ�˲���ʼ��
 *@param	Kalman_data_t *kf �������ṹ�����
 *@return	none
 */
void LP_FilterInit(lpf_data_t *lp,float *fliter_num)
{
	lp->fliter_num[0] = fliter_num[0];
	lp->fliter_num[1] = fliter_num[1];
	lp->fliter_num[2] = fliter_num[2];
}

/**
 *@brief	���׵�ͨ�˲�
 *@param	hpf_data_t *hp ,��ͨ�˲��ṹ�����;input ,����ֵ
 *@return	�˲���Ĳ���
 */
float LP_FilterCalc(lpf_data_t *lp,float input)
{
	lp->in = input;
	lp->fliter_1 = lp->fliter_2;
	lp->fliter_2 = lp->fliter_3;
	lp->fliter_3 = lp->fliter_2 * lp->fliter_num[0] + lp->fliter_1 * lp->fliter_num[1] +  input * lp->fliter_num[2];
	lp->out=lp->fliter_3;
	return lp->out;
}



/**
 *@brief	�������˲�����ʼ��
 *@param	Kalman_data_t *kf �������ṹ�����
 *@return	none
 */
void Kalman_Init(kf_data_t *kf,float Q,float R)
{
	kf->Last_P = 1;			
	kf->Now_P = 0;		
	kf->out = 0;			
	kf->Kg = 0;		
	kf->Q = Q;
	kf->R = R;
}

/**
 *@brief	�������˲���
 *@param	Kalman_data_t *kf ,�������ṹ�����;input ,����ֵ
 *@return	�˲���Ĳ���
 */
float KalmanFilterCalc(kf_data_t *kf,float input)
{
   //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
   kf->Now_P = kf->Last_P + kf->Q;
		kf->ek = 1 * kf->out;
		kf->ek = input - kf->ek;
		kf->ek = kf->ek * 1.0f/(kf->Now_P + kf->R);
		kf->rk = kf->ek * kf->ek;
   //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
   kf->Kg = kf->Now_P / (kf->Now_P + kf->R);
   //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
   kf->out = kf->out + kf->Kg * (input -kf->out);//��Ϊ��һ�ε�Ԥ��ֵ������һ�ε����ֵ
   //����Э�����: ���ε�ϵͳЭ����� kf->LastP ����һ������׼����
   kf->Last_P = (1-kf->Kg) * kf->Now_P;
   return kf->out;
}


