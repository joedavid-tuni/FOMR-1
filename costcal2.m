function sc=costcal2(a,b)

n=size(b,1);
df=repmat(a,n,1)-b;
df(:,3)=abs(df(:,3));
I=find(df(:,3)>pi);
df(I,3)=2*pi-df(I,3);
c=df(:,1).*df(:,1)+df(:,2).*df(:,2);
sc=sqrt(sum(c,2))+0*df(:,3)+0;
%sc=ones(n,1);