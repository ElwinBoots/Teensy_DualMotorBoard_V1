% file = 'C:\Users\Elwin\Documents\Arduino\teensy40_pololuG2test2binary11_pmsm_ADCtest7\defines.h';
file = 'C:\GIT\Teensy_DualMotorBoard_V1\defines.h';

h = fopen(file);
text = fscanf( h , '%c');
fclose( h );

textsplit = strsplit( text , {' ' , ';' , '\n' });
textsplit( find(contains(textsplit , '[]'))-1) = [];

float = {};
uint = {};
int = {};
bool = {};
floatindex = [];
uintindex = [];
intindex = [];
boolindex = [];

for i = 1:length(textsplit)
    if(strcmp(textsplit{i} , 'float' ))
        float = [float textsplit{i+1}];
        if(strcmp(textsplit{i-1} ,'const'))
            floatindex(length(float)) = 1;
        else
            floatindex(length(float)) = 0;
        end
    end
    if(strcmp(textsplit{i} , 'int' ))
        if( i-1 > 1 && strcmp(textsplit{i-1} , 'unsigned' ))
            uint = [uint textsplit{i+1}];
            if(strcmp(textsplit{i-2} ,'const'))
                uintindex(length(uint)) = 1;
            else
                uintindex(length(uint)) = 0;
            end
        else
            int = [int textsplit{i+1}];
            if(strcmp(textsplit{i-1} ,'const'))
                intindex(length(int)) = 1;
            else
                intindex(length(int)) = 0;
            end
        end
    end
    if(strcmp(textsplit{i} , 'bool' ))
        bool = [bool textsplit{i+1}];
        if(strcmp(textsplit{i-1} ,'const'))
            boolindex(length(bool)) = 1;
        else
            boolindex(length(bool)) = 0;
        end
    end
end

txt = { 'void trace( ) {'};
txt = [txt '  for( int i = 0; i<14; i++){'];
txt = [txt '    int isignal = tracearray[i];'];
jsignal = 0;
txt = [txt ['    switch( isignal ){' ]];
for i=1:length(float)
    txt = [txt ['      case ' sprintf( '%3i' ,jsignal) ': bf.fp   = ' float{i} '; break;']];
    jsignal = jsignal + 1;
    signaltype( jsignal ) = 'f';
end
for i=1:length(int)
    txt = [txt ['      case ' sprintf( '%3i' ,jsignal) ': bf.sint = ' int{i} '; break;']];
    jsignal = jsignal + 1;
    signaltype( jsignal ) = 'i';
end
for i=1:length(uint)
    txt = [txt ['      case ' sprintf( '%3i' ,jsignal) ': bf.uint = ' uint{i} '; break;']];
    jsignal = jsignal + 1;
    signaltype( jsignal ) = 'I';
end
for i=1:length(bool)
    txt = [txt ['      case ' sprintf( '%3i' ,jsignal) ': bf.bl   = ' bool{i} '; break;']];
    jsignal = jsignal + 1;
    signaltype( jsignal ) = 'b';
end

txt = [txt ['    }' ]];
txt = [txt  '    Serial.write( bf.bin , 4);'];
txt = [txt '  }'];
txt = [txt ['}' newline]];

%% 
txt = [txt 'void setpar( int isignal , binaryFloat bf ) {'];
jsignal = 0;
txt = [txt ['  switch( isignal ){' ]];
for i=1:length(float)
    if( floatindex(i) == 0 )
        txt = [txt ['    case ' sprintf( '%3i' ,jsignal) ': ' float{i} ' = bf.fp; break;']];
    end  
    jsignal = jsignal + 1;
end
for i=1:length(int)
    if( intindex(i) == 0 )
        txt = [txt ['    case ' sprintf( '%3i' ,jsignal) ': ' int{i} ' = bf.sint; break;']];
    end
    jsignal = jsignal + 1;
end
for i=1:length(uint)
    if( uintindex(i) == 0 )
        txt = [txt ['    case ' sprintf( '%3i' ,jsignal) ': ' uint{i} ' = bf.uint; break;']];
    end
    jsignal = jsignal + 1;
end
for i=1:length(bool)
    if( boolindex(i) == 0 )
        txt = [txt ['    case ' sprintf( '%3i' ,jsignal) ': ' bool{i} ' = bf.bl; break;']];
    end
    jsignal = jsignal + 1;
end

txt = [txt '  }'];
txt = [txt ['}' newline]];

%%

txt = [txt 'void printSignals( unsigned int selected ) {'];
txt = [txt ['  char *signalNames[] = { ' sprintf( '"%s", ' , float{:} , int{:} , uint{:}  , bool{:} ) ' };']];
txt = [txt ['  char *signalTypes[] = { ' sprintf( '"%c", ' , signaltype ) ' };']];

txt = [txt '  int imax = 10;'];
txt = [txt '  switch(selected){'];
txt = [txt ['    case 0: imax = ' int2str(jsignal) '; break;' ]];
txt = [txt '  }'];

%%
txt = [txt '  for ( int i = 0; i < imax; i++) {'];
txt = [txt '    Serial.println( signalNames[i] );'];
txt = [txt '    Serial.println( signalTypes[i] );'];
txt = [txt '  }'];
txt = [txt '}'];

  
filename = 'test.c';
% Write to file
fid = fopen(filename,'w');
for kLine = 1 : length(txt)
    fprintf(fid, '%s\n', txt{kLine});
end
fclose all ;
disp('finished')

