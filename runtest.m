function runtest 
    %this reformatted is the initial estimate
    [h01, h12, h23, h34, h45, h56] = loadMatFromOpenCVXML('test_set_9/homografia', 1, 6);
    
    h16 = importXMLtoMATLAB('testset9test2/homografia_new0006.xml');
end