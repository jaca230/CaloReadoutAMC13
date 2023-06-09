/**
 * @file    frontends/FaceCalo/simulator.cpp
 * @author  Vladimir Tishchenko <tishenko@pa.uky.edu>
 * @date    Thu May 24 16:30:49 2012
 * @date    Last-Updated: Tue Apr  1 18:42:22 2014 (-0400)
 *          By : Data Acquisition
 *          Update #: 219 
 * @version $Id$
 * 
 * @copyright (c) new (g-2) collaboration
 * 
 * 
 * @page    simulator.c
 * 
 * @brief   Simulate calorimeter data
 * 
 * @section Changelog
 * @verbatim
 * $Log$
 * @endverbatim
 */


#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>

// ROOT includes
#include <TRandom.h>
#include <TMath.h>
#include <TF2.h>
#include <TFile.h>
#include <TH1D.h>

#include "simulator.h"


/**
 * Simulator thread info
 */ 
CALO_SIMULATOR_THREAD_INFO calo_simulator_thread_info = 
  {
    0,
    PTHREAD_MUTEX_INITIALIZER,
    PTHREAD_MUTEX_INITIALIZER,
    NULL,
    0
  };

#define CALO_N_SEGMENTS_X  9
#define CALO_N_SEGMENTS_Y  6
// Side lenght of one calorimeter segment, cm
#define CALO_SEGMENT_SIZE  3
#define CALO_N_SEGMENTS    CALO_N_SEGMENTS_X*CALO_N_SEGMENTS_Y
#define WAVEFORM_LENGTH    368640 
#define ADC_TYPE           u_int16_t


/*-- Function declaration -----------------------------------------------------*/
static void *calo_simulator_thread(void *param);
static void generate_events();
static Double_t func_gaus_2D(Double_t *x, Double_t *par);

/*-- Static variables ---------------------------------------------------------*/
static TRandom *random1;
static TF2 *f_gaus_2D;

/** 
 * Called when frontend starts.
 *
 * Creates simulator thread
 * 
 * @return 0 if success
 */

int calo_simulator_init()
{

  // create 2D gaussian function
  // I assume calorimeter dimensions: X = 7x3 = 21 cm; Y = 5x3 = 15 cm
  f_gaus_2D = new TF2("f_gaus_2D",&func_gaus_2D,
		      0.0,CALO_SEGMENT_SIZE*CALO_N_SEGMENTS_X,
		      0.0,CALO_SEGMENT_SIZE*CALO_N_SEGMENTS_Y,
		      3);
  // set Moliere radius
  f_gaus_2D->SetParameter(2,1.5);

  /** Trigger thread **/
  calo_simulator_thread_info.data_size = CALO_N_SEGMENTS*WAVEFORM_LENGTH; // 54 calorimeters x 500 MHz x 737 us
  int buf_size = calo_simulator_thread_info.data_size * sizeof( ADC_TYPE ); 
  calo_simulator_thread_info.data = (ADC_TYPE*) malloc( buf_size );
  if ( calo_simulator_thread_info.data == NULL )
    {
      printf("***ERROR! Cannot allocate memory for simulator data");
      return -1;
    }
  
  pthread_mutex_lock( &(calo_simulator_thread_info.mutex)  );
  pthread_create(&calo_simulator_thread_info.thread_id, 
		 NULL, 
		 calo_simulator_thread, 
		 (void *)(&calo_simulator_thread_info) );

  
  random1 = new TRandom();
  
  return 0;
  
}


/*-- Main thread --------------------------------------------------------------*/
void *calo_simulator_thread(void *param)
{
  
  //CALO_SIMULATOR_THREAD_INFO *info = (CALO_SIMULATOR_THREAD_INFO*) param;
  
  //u_int16_t         *data = info->data;
  ADC_TYPE         *data = calo_simulator_thread_info.data;
  
  printf("Calo simulator thread started\n");
  
  while (1)
    {
      pthread_mutex_lock( &(calo_simulator_thread_info.mutex)  );  // will be unlocked by MIDAS
      pthread_mutex_lock( &(calo_simulator_thread_info.mutex_data)  );
      
      printf("Calo simulator thread unlocked\n");

#if 1
      static int first = 0;
      if ( first == 0 )
	{
	  // use MC event generator
	  generate_events();
	  first = 1;
	}
#else
      generate_events();

      /*
      // simple data for testing 

      for (unsigned int i=0; i<calo_simulator_thread_info.data_size; i++)
	{
	  data[i] = 4000; 
	}

      // Make one pulse in one calorimeter detector

      for (unsigned int i=0; i<100; i++)
	{
	  
	  data[i*1000+100] = 3000;
	  data[i*1000+101] = 2000;
	  data[i*1000+102] = 3000;

	}

      //usleep(200000);
      //sleep(2);
      */
#endif      
      


      printf("Calo simulator thread finished\n");
      pthread_mutex_unlock( &(calo_simulator_thread_info.mutex_data)  );
      
      
    }

  return param;
}


/*-- 2D function to simulate positron hit -----------------------------------*/
Double_t func_gaus_2D(Double_t *x, Double_t *par)
{
  
  Double_t x0    = par[0];
  Double_t y0    = par[1];
  Double_t sigma = par[2];

  Double_t X = x[0];
  Double_t Y = x[1];

  Double_t dx = X - x0;
  Double_t dy = Y - y0;

  Double_t r2 = dx*dx + dy*dy;

  Double_t f = exp( - r2 / sigma / sigma );

  return f;

}

/*-- event generator --------------------------------------------------------*/
void generate_events()
{
  
  ADC_TYPE *data = calo_simulator_thread_info.data;
  
  // =============================================
  // PARAMETERS
  // =============================================

  // the average number of muons per fill per calorimenter
   const int n_muons_mean = 400; // nominal value
 
  // generate pedestals
  for (unsigned int i=0; i<calo_simulator_thread_info.data_size; i++)
    {
      //data[i] = 4000; 
      data[i] = 4000 + random1->Gaus(0, 5);
    }

  for (int i_muon=0; i_muon<n_muons_mean; i_muon++)
    {

      // simulate the muon decay time
      const double mu_tau = 64e-6;      // muon lifetime, s
      double t = random1->Exp(mu_tau);  // muon decay time in s
      int t_ct = int(t * 500e6);        // muon decay time in clock ticks
      if ( t_ct > WAVEFORM_LENGTH ) continue;
      
      // spin precession frequency
      const double omega_a = 1.438e6;   // Rad/s

      // simulate the decay energy
      const double Emax = 52.8; // max. positron energy in CM, MeV
      double E = 0;             // positron energy
      double y = 0;             // y = E / Emax
      double A = 0;             // decay asymmetry
      y = random1->Rndm();
      
      A = (2.0*y-1)/(3.0-2.0*y);
      double n = y*y*(3.0 - 2.0*y);	  
      double r_test = n*(1.0+A*TMath::Cos(omega_a*t))*0.5;
      
      double r = random1->Rndm();
      if ( r < r_test )
	{
	  E = y * Emax;
	}
      else
	{
	  continue;
	}
            
      // simulate the decay angle
      double theta = 0; // decay angle
      while ( 1 )
	{
	  theta = random1->Rndm()*TMath::Pi();
	  r_test = random1->Rndm()*1.3;
	  if ( r_test < (1.0+A*cos(theta))*sin(theta) )
	    {
	      break;
	    }
	}

      const double Elab_max = 3.1; // GeV
      double Elab = Elab_max*y*(1.0+TMath::Cos(theta))*0.5;

      // simulate hit coordinates
      // uniform in x (in horisontal plane)
      Double_t x_hit = random1->Rndm()*CALO_SEGMENT_SIZE*CALO_N_SEGMENTS_X;
      //Double_t x_hit = random1->Gaus(0.5*CALO_SEGMENT_SIZE*CALO_N_SEGMENTS_X,3.0);
      // gaussian in y (in vertical plane)
      Double_t y_hit = random1->Gaus(0.5*CALO_SEGMENT_SIZE*CALO_N_SEGMENTS_Y,4.0);

#if 0
      // test
      Elab = 2.5;
      //x_hit = 5.5;
      //y_hit = 3.5;
      t_ct = (i_muon+1)*200;
#endif
      
      f_gaus_2D->SetParameter(0,x_hit);
      f_gaus_2D->SetParameter(1,y_hit);

      

      // integrate over segments
      for (int ix=0; ix<7; ix++)
	for (int iy=0; iy<5; iy++)
	  {
	    
	    Double_t I = f_gaus_2D->Integral(ix*CALO_SEGMENT_SIZE,(ix+1)*CALO_SEGMENT_SIZE,
					     iy*CALO_SEGMENT_SIZE,(iy+1)*CALO_SEGMENT_SIZE);

	    int wf_offset = (ix+iy*CALO_N_SEGMENTS_X)*WAVEFORM_LENGTH;
	      
	    for (int k=-8; k<=8; k++)
	      {
		int kk = k+t_ct;
		if ( kk < 0 || kk >= WAVEFORM_LENGTH ) continue;
		int adc = data[kk+wf_offset];
		adc -= I*TMath::Gaus(k,0.0,2.0)*300.0*Elab;
		if ( adc < 0 ) adc = 0;
		data[kk+wf_offset] = adc;
	      }
	  }
      
    }

#if 0
  // save traces for the presentation
  static int xxx = 0;
  if ( xxx == 0 )
    {
      xxx = 1;
      TFile *fout = new TFile("waveforms.root","ReCreate");
      fout->cd();
      for (int idet=0; idet<35; idet++)
	{
	  TH1D *h1 = new TH1D(Form("h1_segment_%i",idet+1),Form("segment %i",idet+1),WAVEFORM_LENGTH,0.0,2.0*WAVEFORM_LENGTH);
	  for (int j=0; j<368640; j++)
	    {
	      h1->SetBinContent(j+1, data[idet*WAVEFORM_LENGTH+j]);
	    }
	  h1->Write();
	}
      fout->Write();
      delete fout;
    }
#endif

  

}

