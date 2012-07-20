function [x x0] = runtest(testset, test_name, detthresh, changethresh,...
    entry33thresh, smallsmallthresh)
    basefile = sprintf('%s/homografia', testset);
    [h01, h12, h23, h34, h45, h56] = loadMatFromOpenCVXML(basefile, 1, 6);
    basefile2 = sprintf('%s/homografia_new0006.xml', test_name);
    h16 = importXMLtoMATLAB(basefile2);
    h12vec = reshape(h12', 9, 1);
    h23vec = reshape(h23', 9, 1);
    h34vec = reshape(h34', 9, 1);
    h45vec = reshape(h45', 9, 1);
    h56vec = reshape(h56', 9, 1);
    %intial estimate
    x0 = [h12vec(1:8); h23vec(1:8); h34vec(1:8); h45vec(1:8); h56vec(1:8)];
    [x val] = callObjConstr(x0, h16, detthresh, changethresh, ...
        entry33thresh, smallsmallthresh);
    vecToOpenCVXML(x, test_name);
end