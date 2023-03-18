%reading the two audios from the device to MATLAB
[d1, fs1] = audioread('Short_BBCArabic2.wav');
[d2, fs2] = audioread('Short_SkyNewsArabia.wav');

%Converting the audio from stereo (two channels) to monophonic (one channel)
%by adding the two channels
signal1 = d1(:,1)+ d1(:,2);
signal2 = padarray(d2(:,1)+ d2(:,2), 28672,0,'post');

%increase the sampling frequency to be greater than double the maximum 
%cosine frequency to be sampled without also losing any data
signal1 = interp(signal1, 10);
signal2 = interp(signal2, 10);

%initializing the frequencies of the signals
fs = fs1 * 10;
fn1 = 100000;
fn2 = 150000;

%get the time by a step equal to 1/fs and set the final time to have
%exactly the same sampling number
t = 0:1/fs:16.79238;

%compute the value of the two carriers
carrier1 = (cos(2*pi*fn1*t))';
carrier2 = (cos(2*pi*fn2*t))';

%generating the transmitted signal by multipling each signal by its carrier 
%and get their sum (FDM signal) in AM modulator step
Tx_message = signal1.*carrier1 + signal2.*carrier2;

%plotting the fft of the transmitting message
freq_Txmessage = fft(Tx_message);
s = size(freq_Txmessage);
N = s(1);
k = -N/2:N/2-1;
figure
plot(k*fs/N, abs(fftshift(freq_Txmessage)))
title('Transmitted message in F-domain (AM-stage)')
xlabel('F(Hz)')

%Designing the BPF in the RF stage
BPF_param = fdesign.bandpass('Fst1,Fp1,Fp2,Fst2,Ast1,Ap,Ast2', ... 
                             85000, 90000, 110000, 115000, 60, 1, 60, fs);                             
BandPassFilt_RF = design(BPF_param, 'equiripple');
filtered_Rxmessage = filter(BandPassFilt_RF,Tx_message);
figure
plot(k*fs/N, abs(fftshift(fft(filtered_Rxmessage))));
title('Filtered received message in F-domain (RF-stage)')
xlabel('F(Hz)')

%moving the received message to an intermediate frequency in the IF stage
%without RF filter
fn3 = 125000;
Rx_oscillator = (cos(2*pi*fn3*t))';
IF_messageNoFilter = Tx_message.*Rx_oscillator;
figure
plot(k*fs/N, abs(fftshift(fft(IF_messageNoFilter))))
title('Received signal at IF in F-domain without RF Filter (IF-stage)')
xlabel('F(Hz)')

%moving the received message to an intermediate frequency in the IF stage
%with RF filter
IF_message = filtered_Rxmessage.*Rx_oscillator;
figure
plot(k*fs/N, abs(fftshift(fft(IF_message))))
title('Received signal at IF in F-domain with RF Filter (IF-stage)')
xlabel('F(Hz)')

%Desinging new BPF to get the signal
IF_BPFparam = fdesign.bandpass('Fst1,Fp1,Fp2,Fst2,Ast1,Ap,Ast2', ... 
                               10000, 15000, 35000, 40000, 60, 1, 60, fs);
BandPassFilt_IF = design(IF_BPFparam, 'equiripple');
filtered_IFmessage = filter(BandPassFilt_IF,IF_messageNoFilter);
figure
plot(k*fs/N, abs(fftshift(fft(filtered_IFmessage))))
title('Received signal after IF BPF in F-domain (IF BPF-stage)')
xlabel('F(Hz)')

%The baseband demodulator stage
fn4 = 25000;
IF_oscillator = (cos(2*pi*fn4*t))';
BB_message = filtered_IFmessage.*IF_oscillator;
figure
plot(k*fs/N, abs(fftshift(fft(BB_message))))
title('Demodulating signal in BB in F-domain (BB demodulator-stage)')
xlabel('F(Hz)')

%Low pass filter to get the final signal
LPF_param = fdesign.lowpass('Fp,Fst,Ap,Ast',10000,12500,1,60,fs);
LowPassFilt = design(LPF_param, 'equiripple');
final_message = filter(LowPassFilt, BB_message);
figure
plot(k*fs/N, abs(fftshift(fft(final_message))))
title('Final received message in F-domain (final-stage)')
xlabel('F(Hz)')

%downsampling the signal to can hear it
downsampled_message = downsample(final_message,10);

sound (downsampled_message , 44100)