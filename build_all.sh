docker pull quay.io/pypa/manylinux_2_34_x86_64
docker build -f Dockerfile.centos . -t livarot
docker run -it --name livarot livarot
docker cp livarot:/livarot/wheelhouse .
 