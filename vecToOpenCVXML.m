function vecToOpenCVXML(x, directory)
    
%    h12newvec = [x(1:8); 1];
%    writetoFile(2, h12newvec, directory);    
%    h23newvec = [x(9:16); 1];
%    writetoFile(3, h23newvec, directory);
%    h34newvec = [x(17:24); 1];
%    writetoFile(4, h34newvec, directory);
%    h45newvec = [x(25:32); 1];
%    writetoFile(5, h45newvec, directory);
%    h56newvec = [x(33:40); 1];
%    writetoFile(6, h56newvec, directory);

h0001newvec = [x(1:8); 1];
writetoFile(1, h0001newvec, directory);
h0102newvec = [x(9:16); 1];
writetoFile(2, h0102newvec, directory);
h0203newvec = [x(17:24); 1];
writetoFile(3, h0203newvec, directory);
h0304newvec = [x(25:32); 1];
writetoFile(4, h0304newvec, directory);
h0405newvec = [x(33:40); 1];
writetoFile(5, h0405newvec, directory);
h0506newvec = [x(41:48); 1];
writetoFile(6, h0506newvec, directory);
h0607newvec = [x(49:56); 1];
writetoFile(7, h0607newvec, directory);
h0708newvec = [x(57:64); 1];
writetoFile(8, h0708newvec, directory);
h0809newvec = [x(65:72); 1];
writetoFile(9, h0809newvec, directory);
h0910newvec = [x(73:80); 1];
writetoFile(10, h0910newvec, directory);
h1011newvec = [x(81:88); 1];
writetoFile(11, h1011newvec, directory);
h1112newvec = [x(89:96); 1];
writetoFile(12, h1112newvec, directory);
h1213newvec = [x(97:104); 1];
writetoFile(13, h1213newvec, directory);
h1314newvec = [x(105:112); 1];
writetoFile(14, h1314newvec, directory);
h1415newvec = [x(113:120); 1];
writetoFile(15, h1415newvec, directory);
h1516newvec = [x(121:128); 1];
writetoFile(16, h1516newvec, directory);


end

function writetoFile(num, vec, directory)
    filebase = 'homografia_opt';
    thisfilebase = sprintf('%s%04d', filebase, num);
    thisfile = sprintf('%s/%s.xml', directory, thisfilebase);
    fileID = fopen(thisfile, 'w');
    fprintf(fileID, ...
        '<?xml version=\"1.0\"?>\n<opencv_storage>\n<%s type_id=\"opencv-matrix\">\n<rows>3</rows>\n<cols>3</cols>\n<dt>f</dt>\n<data>\n%e %e %e %e\n%e %e %e %e %e </data></%s>\n</opencv_storage>', ...
        thisfilebase, vec, thisfilebase);
    fclose(fileID);
end