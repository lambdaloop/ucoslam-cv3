if [ $# -ne 2 ]; then
  echo "Usage: distro (bionic|xenial) ppaVersion (eg ppa0|ppa1...)"
  exit 1
fi
VERSION=$(cat CMakeLists.txt  | grep "ucoslam VERSION" | cut -f2 -d '"');
mkdir debian
echo "ucoslam ($VERSION-$2ppa$2-$1) $1; urgency=medium" > debian/changelog
echo "">>debian/changelog
echo "   * New version of the library $VERSION">>debian/changelog
echo "">>debian/changelog
date=$(date -R)
echo " -- Rafael Muñoz Salinas <rmsalinas@uco.es>  $date">>debian/changelog


debuild -S -sa

echo " now do: cd .. && dput ppa:rmsalinas/ucoslam ucoslam_$VERSION-$2ppa$2-$1_source.changes"
