function vecToOpenCVXML(x, directory)
    
    h12newvec = [x(1:8); 1];
    writetoFile(2, h12newvec, directory);
    
    h23newvec = [x(9:16); 1];
    writetoFile(3, h23newvec, directory);
    
    h34newvec = [x(17:24); 1];
    writetoFile(4, h34newvec, directory);
    
    h45newvec = [x(25:32); 1];
    writetoFile(5, h45newvec, directory);
    
    h56newvec = [x(33:40); 1];
    writetoFile(6, h56newvec, directory);
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