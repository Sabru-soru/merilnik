#include <stdio.h>
#include <string>
#include <opencv\cv.h>
#include <opencv\highgui.h>

using namespace std;
using namespace cv;

#define DEG2RAD			(0.017453292519943295769236907684886)

struct TransformationParameters
{
public:
	float alpha;	//triangulation angle
	float Py;		//distance between projector and camera in y direction
	float f;		//focal length of the camera lens
	int cu;			//center of the image in U direction
	int cv;			//center of the image in V direction
	float du;		//size of sensor element in U direction
	float dv;		//size of sensor element in V direction
	float dx;		//step between two consequentional images

	TransformationParameters(){
		alpha = Py = f = du = dv = dx = 0;
	}

	TransformationParameters(float alpha, float Py, float f, int cu, int cv, float du, float dv, float dx){
		this->alpha = alpha;
		this->Py = Py;
		this->f = f;
		this->cu = cu;
		this->cv = cv;
		this->du = du;
		this->dv = dv;
		this->dx = dx;
	}
	~TransformationParameters(){}

};

void SaveFloatMtrx(string filename, Mat *input){
	long LofPrf = input->cols;
	long NofPrf = input->rows;

	FILE *stream = fopen(filename.data(),"w");
	float* pData;
	for(int v=0;v<NofPrf;v++){
		pData = &input->at<float>(v,0);
		for(int u=0;u<LofPrf-1; u++, pData++){
			fprintf(stream,"%f ",*pData);
		}
		if(v != NofPrf-1)	fprintf(stream,"%f\n",*(pData+1));
	}

	fclose(stream);
}

void SaveUCharMtrx(string filename, Mat *input){
	long LofPrf = input->cols;
	long NofPrf = input->rows;

	FILE *stream = fopen(filename.data(),"w");
	unsigned char* pData;
	for(int v=0;v<NofPrf;v++){
		pData = &input->at<unsigned char>(v,0);
		for(int u=0;u<LofPrf-1; u++, pData++){
			fprintf(stream,"%d ",*pData);
		}
		if(v != NofPrf-1)	fprintf(stream,"%d\n",*(pData+1));
	}

	fclose(stream);
}

void SaveMXYZfile(string filename, Mat *x, Mat *y, Mat *z){
	long LofPrf = x->cols;
	long NofPrf = x->rows;

	FILE *stream = fopen(filename.data(),"w");
	float *pX,*pY,*pZ;
	for(int v=0;v<NofPrf;v++){
		pX = &x->at<float>(v,0);
		pY = &y->at<float>(v,0);
		pZ = &z->at<float>(v,0);
		for(int u=0;u<LofPrf; u++, pX++){
			fprintf(stream,"%f\t",*pX);
		}
		for(int u=0;u<LofPrf; u++, pY++){
			fprintf(stream,"%f\t",*pY);
		}
		for(int u=0;u<LofPrf-1; u++, pZ++){
			fprintf(stream,"%f\t",*pZ);
		}
		if(v != NofPrf-1)	fprintf(stream,"%f\n",*(pZ+1));
		else	fprintf(stream,"%f",*(pZ+1));
	}

	fclose(stream);
}

namespace dt{

	class Vec;

	float operator*(const Vec &a, const Vec &b);
	Vec cross(const Vec &a, const Vec &b);
	Vec operator^(const Vec &a, const Vec &b);
	Vec operator+(const Vec &a, const Vec &b);
	Vec operator-(const Vec &a, const Vec &b);
	Vec operator-(const Vec &a);
	Vec operator*(const Vec &a, const float k);
	Vec operator*(float k, const Vec &a);
	Vec operator/(const Vec &a, const float k);
	bool operator!=(const Vec &a, const Vec &b);
	bool operator==(const Vec &a, const Vec &b);

	class  Vec
	{
	public:
		//! The internal data representation is public. One can use v.x, v.y, v.z.
		float x, y, z;

		//! Default constructor. Default value is (0,0,0).
		explicit Vec() : x(0.0), y(0.0), z(0.0) {}

		//! Standard constructor with the x,y and z values.
		explicit Vec(const float X, const float Y, const float Z) : x(X), y(Y), z(Z) {}

		/*! Universal explicit converter from any class to Vec. You can use your own vector class everywhere
		a \p const \p Vec& parameter is required, as long as it implements the [] operator.
		\code
		// Declared in class MyVec
		float operator[](int i) const { return (x, y or z); }

		// Then you can use
		MyVec v(...);
		camera()->setPosition( qglviewer::Vec(v) );
		\endcode
		*/
		template <class C>
		explicit Vec(const C& c) : x(c[0]), y(c[1]), z(c[2]) {}

		// ! Copy constructor
		// explicit Vec(const Vec& v) : x(v.x), y(v.y), z(v.z) {}

		//! Classical = operator
		Vec& operator=(const Vec& v)
		{
			x = v.x;   y = v.y;   z = v.z;
			return *this;
		}

		//! Set the current value. Faster than using operator equal with a temporary Vec(x,y,z).
		void setValue(const float X, const float Y, const float Z)
		{ x=X; y=Y; z=Z; }

		// Universal equal operator which allows the use of any type in place of Vec,
		// as long as the [] operator is implemented (v[0]=v.x, v[1]=v.y, v[2]=v.z).
		// template <class C>
		// Vec& operator=(const C& c)
		// {
		// x=c[0]; y=c[1]; z=c[2];
		// return *this;
		// }

		//! Bracket operator, with a constant return value
		float operator[](int i) const { return (&x)[i]; }

		//! Bracket operator, returns an l-value
		float& operator[](int i) { return (&x)[i]; }

		//! The memory address of the vector. Useful as an argument for glVertex3fv, glNormal3fv...
		const float* address() const { return &x; };

		//! Dereferencing operator that returns the memory address of the vector. Same as address().
		operator const float*() const { return &x; };
		//@}

		/*! @name Calculus */
		//@{
		//! Returns the sum of the two vectors.
		friend Vec operator+(const Vec &a, const Vec &b)
		{
			return Vec(a.x+b.x, a.y+b.y, a.z+b.z);
		}

		//! Returns the difference of the two vectors.
		friend Vec operator-(const Vec &a, const Vec &b)
		{
			return Vec(a.x-b.x, a.y-b.y, a.z-b.z);
		}

		//! Unary minus operator
		friend Vec operator-(const Vec &a)
		{
			return Vec(-a.x, -a.y, -a.z);
		}

		//! Returns the product of the vector with a scalar.
		friend Vec operator*(const Vec &a, const float k)
		{
			return Vec(a.x*k, a.y*k, a.z*k);
		}

		//! Returns the product of the vector with a scalar.
		friend Vec operator*(float k, const Vec &a)
		{
			return Vec(a.x*k, a.y*k, a.z*k);
		}

		//! Returns the division of the vector with a scalar. If the library was compiled with the "debug" qt \p CONFIG flag, tests for null value.
		friend Vec operator/(const Vec &a, const float k)
		{
			/*#ifndef QT_NO_DEBUG
			if (fabs(k) < 1.0E-10)
			qWarning("Vec::operator / : dividing by a null value");
			#endif
			*/    return Vec(a.x/k, a.y/k, a.z/k);
		}

		//! Comparison based on the squared norm of the difference vector, see operator==.
		friend bool operator!=(const Vec &a, const Vec &b)
		{
			return !(a==b);
		}

		//! Comparison based on the \e squared norm of the difference vector, epsilon=1E-10.
		friend bool operator==(const Vec &a, const Vec &b)
		{
			const float epsilon = 1.0E-10f;
			Vec c(a-b);
			return (c.sqNorm() < epsilon);
		}

		//! Adds \p a to the vector.
		Vec& operator+=(const Vec &a)
		{
			x += a.x; y += a.y; z += a.z;
			return *this;
		}

		//! Subtracts \p a to the vector.
		Vec& operator-=(const Vec &a)
		{
			x -= a.x; y -= a.y; z -= a.z;
			return *this;
		}

		//! Multiply the vector by a scalar.
		Vec& operator*=(float k)
		{
			x *= k; y *= k; z *= k;
			return *this;
		}

		//! Divides the vector by a scalar. If the library was compiled with the "debug" qt \p CONFIG flag, tests for null value.
		Vec& operator/=(float k)
		{
			/*#ifndef QT_NO_DEBUG
			if (fabs(k)<1.0E-10)
			qWarning("Vec::operator /= : dividing by a null value");
			#endif
			*/    x /= k; y /= k; z /= k;
		return *this;
		}

		//! Dot product.
		friend float operator*(const Vec &a, const Vec &b)
		{
			return a.x*b.x + a.y*b.y + a.z*b.z;
		}

		//! Cross product of the two vectors. Mind the order !
		friend Vec cross(const Vec &a, const Vec &b)
		{
			return Vec(a.y*b.z - a.z*b.y,
				a.z*b.x - a.x*b.z,
				a.x*b.y - a.y*b.x);
		}

		//! Cross product of the two vectors. See also cross().  
		friend Vec operator^(const Vec &a, const Vec &b)
		{
			return Vec(a.y*b.z - a.z*b.y,
				a.z*b.x - a.x*b.z,
				a.x*b.y - a.y*b.x);
		}

		//! Returns the \e squared norm of the Vec.
		float sqNorm() const { return x*x + y*y + z*z; }

		//! Returns the norm of the vector.
		float norm() const { return sqrt(x*x + y*y + z*z); }

		//! Normalizes the Vec. Its previous norm is returned. If the library was compiled with the "debug" qt \p CONFIG flag, tests for null value.
		float normalize()
		{
			const float n = norm();
			/*#ifndef QT_NO_DEBUG
			if (n < 1.0E-10)
			qWarning("Vec::normalize : normalizing a null vector");
			#endif
			*/      *this /= n;
			return n;
		}

		//! Returns a unitary (normalized) \e representation of the vector. The original Vec is not modified.
		Vec unit()
		{
			Vec v = *this;
			v.normalize();
			return v;
		}

		void projectOnAxis(const Vec& dir)
		{
			// We assume dir is normalized
			if (fabs(1.0-dir.norm()) > 1.0E-6)
				return;//qWarning("Vec::projectOnAxis : axis direction is not normalized (norm=%f).", dir.norm());
			const float dot = x*dir.x + y*dir.y + z*dir.z ;
			x = dot*dir.x;
			y = dot*dir.y;
			z = dot*dir.z;
		}

		/*! Projects on the plane whose normal is n and that passes through the origin.
		\attention The plane normal vector must be normalized. This is checked in the debug library release. */
		void projectOnPlane(const Vec& n)
		{
			// We assume dir is normalized
			if (fabs(1.0-n.norm()) > 1.0E-6)
				return;//qWarning("Vec::projectOnPlane : plane normal is not normalized (norm=%f).", n.norm());
			const float dot = x*n.x + y*n.y + z*n.z ;

			x -= dot*n.x;
			y -= dot*n.y;
			z -= dot*n.z;
		}
	};

	Vec CalcNormal(Vec T0, Vec T1, Vec T2);
}

class MTriangle
{
public:
	unsigned long T[3];			// indeksi vogalnih tock
	//long E[3];			// indeksi robov
public:
	MTriangle(void)
	{
	}
	~MTriangle(void)
	{
	}
	void Check(dt::Vec *point_cloud, float &smax_2)
	{
		dt::Vec s0 = point_cloud[T[2]]-point_cloud[T[1]];
		dt::Vec s1 = point_cloud[T[0]]-point_cloud[T[2]];
		dt::Vec s2 = point_cloud[T[1]]-point_cloud[T[0]];
		smax_2 = __max(s0.sqNorm(),__max(s1.sqNorm(),s2.sqNorm()));
	}	
	float smax_2(dt::Vec *point_cloud)
	{
		dt::Vec s0 = point_cloud[T[2]]-point_cloud[T[1]];
		dt::Vec s1 = point_cloud[T[0]]-point_cloud[T[2]];
		dt::Vec s2 = point_cloud[T[1]]-point_cloud[T[0]];
		return __max(s0.sqNorm(),__max(s1.sqNorm(),s2.sqNorm()));
	}	

};

long calcNormals(long orientation, vector<dt::Vec> *points, vector<MTriangle> *triangles,  vector<dt::Vec> *normals)
{
	unsigned long i;
	unsigned long NofT = triangles->size();
	unsigned long NofP = points->size();
	normals->resize(NofP);
	MTriangle *pT = &(*triangles)[0];
	dt::Vec *pP = &(*points)[0];
	dt::Vec *pN = &(*normals)[0];
	dt::Vec r1;				// vektor med T0 in T1
	dt::Vec r2;				// vektor med T0 in T2
	dt::Vec tn;				// normala posameznega trikotnika
	unsigned long T0, T1, T2;
	if(pT == NULL)
		return -1;
	if((pN == NULL)||(pP == NULL))
		return -2;

	// 1. resetiranje vrednosti normal:
	memset(pN,0,sizeof(dt::Vec)*NofP);

	// 2. izracun normal trikotnikov in sumiranje normal v tockah:

	for(i=0; i<NofT; i++){
		T0 = pT[i].T[0];
		T1 = pT[i].T[1];
		T2 = pT[i].T[2];
		r1 = pP[T1] - pP[T0];
		r2 = pP[T2] - pP[T0];

		tn.x = r1[1]*r2[2] - r1[2]*r2[1];
		tn.y = r1[2]*r2[0] - r1[0]*r2[2];
		tn.z = r1[0]*r2[1] - r1[1]*r2[0];

		pN[T0] += tn;
		pN[T1] += tn;
		pN[T2] += tn;
	}
	// 4. normalizacija normal:
	for(i=0; i<NofP; i++)
		pN[i].normalize();

	return 0;
}

long removeUnusedPoints(vector<dt::Vec> *points, vector<MTriangle> *triangles)
{
	vector<long> PointNewIndex(points->size());		// novi indeksi 3d tock
	unsigned long *pTri;								// kazalec na trikotnike
	vector<dt::Vec> tmp_points(points->size());			// zacasen buffer za tocke
	dt::Vec *pp;
	unsigned long *pc;
	int i,NofP, NofT,lastPointIndex;
	NofP=points->size();
	NofT=triangles->size();
	// 1. prekopiramo tocke v tmp:
	memcpy(&tmp_points[0], &points[0], 12*NofP);

	// najprej nove indekse resetiramo:
	for(i=0;i<NofP;i++)
		PointNewIndex[i] = -1;	// indikator, da tocka se ni uporabljena

	// 2. Sedaj na novo ostevilcimo uporabljene tocke:
	lastPointIndex = 0;
	pTri = (unsigned long*)&(*triangles)[0];
	for(i=0; i<NofT*3; i++)	{
		if(PointNewIndex[*pTri] == -1)		{
			PointNewIndex[*pTri] = lastPointIndex;
			lastPointIndex++;
		}
		pTri++;
	}
	// 3. sedaj na novo kreiramo trikotnike:
	pTri = (unsigned long*)&(*triangles)[0];
	for(i=0; i<NofT*3; i++)	{
		if(PointNewIndex[*pTri] != -1)
			*pTri = PointNewIndex[*pTri];
		else
			i=i;
		pTri++;
	}
	// 4. vektorju tock in barv popravimo velikost in vanj na novo razporedimo tocke:
	points->resize(lastPointIndex);
	pp = &(*points)[0];
	for(i=0; i<NofP; i++)	{
		if((PointNewIndex[i]) != -1){
			pp[PointNewIndex[i]] = tmp_points[i];
		}
	}
	
	return 0;
}

long generateFromPointCloud(Mat *x, Mat *y, Mat *z, vector<dt::Vec> *points, vector<MTriangle> *triangles, vector<dt::Vec> *normals, float min[], float max[], float smax){
	long i, j, NofP, NofT;//, NofProfiles, ProfileLength;
	long LofPrf = x->cols, NofPrf = x->rows;
	const float *pX, *pY, *pZ;
	float xmin, ymin, zmin, xmax, ymax, zmax;
	xmin = min[0];
	ymin = min[1];
	zmin = min[2];
	xmax = max[0];
	ymax = max[1];
	zmax = max[2];

	MTriangle *pT;
	dt::Vec *pP;
	// alociramo prostor:
	NofP = x->total();
	//NofP = X.size();
	//NofP = P->X.size();
	points->resize(NofP);
	triangles->resize(NofP*2);
	
	pT = &(*triangles)[0];
	pP = &(*points)[0];

	pX = &x->at<float>(0,0);
	pY = &y->at<float>(0,0);
	pZ = &z->at<float>(0,0);

	for (i = 0;i < NofP;i++, pX++,pY++,pZ++,pP++){
		pP->x = *pX;
		pP->y = *pY;
		pP->z = *pZ;
	}

	long icl, inl;						// indeksa tock v trenutni in naslednji vrstici
	NofT = 0;
	pP = &(*points)[0];
	// kriterij najdaljše stranice kvadriramo zaradi hitrejsega procesiranja:
	smax *= smax;
	for(j=0; j<NofPrf-1; j++)
	{	
		icl = j*LofPrf;
		inl = icl+LofPrf;
		for(i=0; i<LofPrf-1; i++)
		{
			//if(j==354) printf("%d ",i);
			// trikotnik A:
			if( (pP[icl].z>zmin)&&(pP[icl+1].z>zmin)&&(pP[inl].z>zmin)&&
				(pP[icl].z<zmax)&&(pP[icl+1].z<zmax)&&(pP[inl].z<zmax)&&
				(pP[icl].x>xmin)&&(pP[icl+1].x>xmin)&&(pP[inl].x>xmin)&&
				(pP[icl].x<xmax)&&(pP[icl+1].x<xmax)&&(pP[inl].x<xmax)&&
				(pP[icl].y>ymin)&&(pP[icl+1].y>ymin)&&(pP[inl].y>ymin)&&
				(pP[icl].y<ymax)&&(pP[icl+1].y<ymax)&&(pP[inl].y<ymax))
			{
				if((pP[icl]-pP[inl]).sqNorm()<smax)
				{
					if((pP[icl+1]-pP[inl]).sqNorm()<smax)
					{
						if((pP[icl]-pP[icl+1]).sqNorm()<smax)
						{
							pT->T[0] = icl;
							pT->T[1] = icl+1;
							pT->T[2] = inl;
							pT++;
							NofT++;
						}
					}
				}
			}
			// trikotnik B:
			if( (pP[inl].z>zmin)&&(pP[icl+1].z>zmin)&&(pP[inl+1].z>zmin)&&
				(pP[inl].z<zmax)&&(pP[icl+1].z<zmax)&&(pP[inl+1].z<zmax)&&
				(pP[inl].x>(-1.0E10))&&(pP[icl+1].x>(-1.0E10))&&(pP[inl+1].x>(-1.0E10))&&
				(pP[inl].y>(-1.0E10))&&(pP[icl+1].y>(-1.0E10))&&(pP[inl+1].y>(-1.0E10)))
			{
				if((pP[inl]-pP[icl+1]).sqNorm()<smax)
				{
					if((pP[icl+1]-pP[inl+1]).sqNorm()<smax)
					{
						if((pP[inl]-pP[inl+1]).sqNorm()<smax)
						{
							pT->T[0] = inl;
							pT->T[1] = icl+1;
							pT->T[2] = inl+1;
							pT++;
							NofT++;
						}
					}
				}
			}
			icl++;
			inl++;
		}
	}
	// na koncu popravimo stevilo trikotnikov:
	if(NofT<1){
		printf("ERROR: Generirtati ni mozno nobenega trikotnika!\n");
		return -111;
	}
	triangles->resize(NofT);
	//removeUnusedPoints(points, triangles);
	// izracun normal:
	calcNormals(0, points,triangles, normals);
	return 0;

}

int SaveVRMLfile(string filename, Mat *x, Mat *y, Mat *z){

	vector<dt::Vec> points;
	vector<dt::Vec> normals;
	vector<MTriangle> triangles;

	float min[3] = {-5000,-5000,-5000};
	float max[3] = {5000,5000,5000};

	generateFromPointCloud(x,y,z,&points,&triangles,&normals,min,max,5.0);

	FILE *stream;
	unsigned long i;
	if( (stream  = fopen( filename.data(), "w" )) == NULL )
		return -1;
	// najprej zapisemo verzijo VRML-ja:
	fprintf(stream, "#VRML V2.0 utf8\n\n");
	// nato nekaj komentarjev:
	fprintf(stream, "#3d furface output");
	fprintf(stream, "#Modul:   MMesh.cpp/saveVRML\n");
	fprintf(stream, "#Author:  Urban Pavlovcic (urban.pavlovcic@fs.uni-lj.si) & Matija Jezersek (matija.jezersek@fs.uni-lj.si)\n");
	fprintf(stream, "#Company: University of Ljubljana, Faculty of Mechanichal Engineering\n");
	fprintf(stream, "#         Chair of optodynamic and laser applications\n");

	// pricnemo s pravo VRML kodo:
	fprintf(stream, "Shape {\n");
	// najprej material in ostalo kar se tice prikaza:
	fprintf(stream, "appearance Appearance {\n");
	fprintf(stream, "material Material {\n");
	fprintf(stream, "ambientIntensity 1.8\n");
	fprintf(stream, "diffuseColor	1.8 1.8 1.8\n");
	fprintf(stream, "specularColor	0 0 0\n");
	fprintf(stream, "emissiveColor	0 0 0\n");
	fprintf(stream, "shininess	0.0015625\n");
	fprintf(stream, "transparency	0\n");
	fprintf(stream, "}\n");	// konec material
	fprintf(stream, "}\n");	// konec appereance

	fprintf(stream, "geometry IndexedFaceSet {\n");
	fprintf(stream, "solid FALSE\n");
	fprintf(stream, "creaseAngle 0.785398 \n");
	// sledi zapis koordinat:
	fprintf(stream, "coord Coordinate {\n");
	fprintf(stream, "point [\n");
	dt::Vec *pp;
	pp = &points[0];
	for(i=0; i<points.size(); i++, pp++)
		fprintf(stream,"%.3f %.3f %.3f,\n", pp->x, pp->y, pp->z);
	// zakljucek pisanja koordinat:
	fprintf(stream, "]\n}\n");
	// pricetek pisanja normal:
	//fprintf(stream, "normal Normal {\n");
	//fprintf(stream, "vector [\n");
	//pp = &normals[0];
	//for(i=0; i<normals.size(); i++, pp++)
	//	fprintf(stream,"%.3f %.3f %.3f,\n", pp->x, pp->y, pp->z);
	//// zakljucek pisanja normal:
	//fprintf(stream, "]\n}\n");
	// pricetek pisanja barv:
	//if(colors.size()==points.size()) {
	//	fprintf(stream, "color Color {\n");
	//	fprintf(stream, "color [\n");
	//	unsigned char *pRed, *pGreen, *pBlue;
	//	pRed = (unsigned char *)&colors[0];
	//	//pRed++;
	//	pGreen = &pRed[1];
	//	pBlue = &pRed[2];
	//	for(i=0; i<colors.size(); i++,  pRed+=4, pGreen+=4, pBlue+=4)
	//		fprintf(stream,"%.3f %.3f %.3f,\n", float(*pBlue)/255.0f, float(*pGreen)/255.0f, float(*pRed)/255.0f);
	//	// zakljucek pisanja barv:
	//	fprintf(stream, "]\n}\n");
	//}
	// pricetek pisanja trikotnikov:
	fprintf(stream, "coordIndex [\n");
	for(i=0; i<triangles.size(); i++)
		fprintf(stream,"%d %d %d -1\n", triangles[i].T[0],triangles[i].T[1],triangles[i].T[2]);
	// zakljucek
	fprintf(stream, "]\n}\n}\n");
	fclose(stream);

	return 0;
}