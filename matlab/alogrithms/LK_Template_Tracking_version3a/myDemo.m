clc
clearvars -except data imFig tempFig

if ~exist('tempFig') | ~exist('imFig')
  close all
  tempFig = figure;
  imFig = figure;
end

if ~exist('data')
  disp('Loading data...');
  file = ['/home/miacono/workspace/DATASETS/objects_contour/train/shaking/atis/data.log.txt'];
  data = dlmread(file);
  disp ('Finished loading!');
end

%Taking just channel 1
ch = logical(data(:,1));
data(ch,:) = [];

%Cropping
minY = 70;
maxY = 180;
minX = 70;
maxX = 200;

%Creating vector of dts
currTs = data (end,2);
dt = (currTs - data(:,2)) * 80e-9;

%Laplacian filter
L = [0  1  0;
     1 -4  1;
     0  1  0];
     
maxVar = [];
step = 0.001;
startTime = 17.0;
duration = 5;

mat = zeros(304,240);

for ts = startTime:step:startTime+duration
  
  %Asking if increasing window size or stop
  windSize = ts - startTime;
  prompt = strcat ( 'Temporal window size = ' , num2str(windSize), '. Continue? [y/n]');
  str = input(prompt,'s');
  if isempty(str)
      str = 'Y';
  end
  if str == 'n'
    break;
  end
  
  %Considering events in temporal window
  inWindow = (dt > startTime) & (dt < ts);
  vWind = data(inWindow,:);
  X = vWind(:,4) + 1; %Indices cannot be 0
  Y = vWind(:,5) + 1;
  
  %Saving events in matrix
  mat = accumarray ([X Y], 1, size(mat));
  maxVal = max(max(mat));
  template = mat(minX:maxX,minY:maxY);
  figure(tempFig)
  imagesc (template,[0,8]); 
end

%Compute metric
%  mat = conv2(mat,L);
%  v = max(var(mat));
%  maxVar = [maxVar; ts v];


TemplateData.image=template;
TemplateData.weight=ones(size(template));
TemplateData.p=[0 0 0 0 0 0];

Options.TranslationIterations=30;
Options.AffineIterations=0;
Options.RoughSigma=3;
Options.FineSigma=1.5;

mat = zeros(304,240);
T_error = zeros( floor(duration/windSize),1);
i=0;
startTime = 3;
duration = 10;

for ts = startTime:windSize:startTime+duration
  %Take events in temporal window
  inWindow = (dt > (ts - windSize)) & (dt < ts);
  vWind = data(inWindow,:);
  X = vWind(:,4) + 1; %Indices cannot be 0
  Y = vWind(:,5) + 1;
  mat = accumarray ([X Y], 1, size(mat));

  figure(imFig);
  hold on;
  imagesc(mat,[0 8]);
  if i == 1
    center = ginput(1);
    TemplateData.p=[0 0 0 0 center(2) center(1)];
  end
  plot(TemplateData.p(6),TemplateData.p(5),'or');
  drawnow;
  hold off;
  
  i=i+1;
  [TemplateData.p,ROIimage,T_error(i)]=LucasKanadeInverseAffine(mat,TemplateData.p,TemplateData.image,TemplateData.weight,Options);
  
  
  
end
