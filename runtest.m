function [x x0] = runtest 
    [h01, h12, h23, h34, h45, h56] = loadMatFromOpenCVXML('test_set_14/homografia', 1, 6);
    h16 = importXMLtoMATLAB('testset14test2/homografia_new0006.xml');
    h12vec = reshape(h12', 9, 1);
    h23vec = reshape(h23', 9, 1);
    h34vec = reshape(h34', 9, 1);
    h45vec = reshape(h45', 9, 1);
    h56vec = reshape(h56', 9, 1);
    %intial estimate
    x0 = [h12vec(1:8); h23vec(1:8); h34vec(1:8); h45vec(1:8); h56vec(1:8)];
    [x val] = callObjConstr(x0)
    vecToOpenCVXML(x, 'testset14test2')
end