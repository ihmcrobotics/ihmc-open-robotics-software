#include<iostream>
#include<sstream>
#include<string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/recognition/linemod.h>
#include <pcl/recognition/color_gradient_modality.h>
#include <pcl/recognition/surface_normal_modality.h>

#ifdef WITH_PNG
#include <png++/png.hpp>
#endif

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudXYZRGBA;

PointCloudXYZRGBA::Ptr xyzrgb2cloud(int w, int h, float* xyzrgb)
{
	const PointCloudXYZRGBA::Ptr input(new PointCloudXYZRGBA(w, h));
	int iter=0;
	for(int i=0;i<input->points.size();i++)
	{
		input->points[i].x   =  xyzrgb[iter++]; 
		input->points[i].y   =  xyzrgb[iter++]; 
		input->points[i].z   =  xyzrgb[iter++]; 
		input->points[i].rgb =  xyzrgb[iter++];
	}
	return  input;
}

std::vector<pcl::LINEMODDetection>
matchTemplates (const PointCloudXYZRGBA::ConstPtr & input, const pcl::LINEMOD & linemod)
{  
  pcl::ColorGradientModality<pcl::PointXYZRGBA> color_grad_mod;
  color_grad_mod.setInputCloud (input);
  color_grad_mod.processInputData ();

  pcl::SurfaceNormalModality<pcl::PointXYZRGBA> surface_norm_mod;
  surface_norm_mod.setInputCloud (input);
  surface_norm_mod.processInputData ();

  std::vector<pcl::QuantizableModality*> modalities (2);
  modalities[0] = &color_grad_mod;
  modalities[1] = &surface_norm_mod;

  std::vector<pcl::LINEMODDetection> detections;
  linemod.matchTemplates (modalities, detections);

  return (detections);
}


extern "C"
{
	SparseQuantizedMultiModTemplate** loadTemplates(char* templates_filename, int* nr_templates)
	{

		std::ifstream file_stream;
		file_stream.open (templates_filename, std::ofstream::in | std::ofstream::binary);
		read(file_stream, *nr_templates);
		SparseQuantizedMultiModTemplate** templates = new SparseQuantizedMultiModTemplate*[*nr_templates];
		for(int i=0;i<*nr_templates;i++)
		{
			templates[i] = new SparseQuantizedMultiModTemplate();
			templates[i]->deserialize(file_stream);
		}
		file_stream.close ();
		return templates;
	}
	

	SparseQuantizedMultiModTemplate* loadTemplate(char* templates_filename)
	{

		std::ifstream file_stream;
		file_stream.open (templates_filename, std::ofstream::in | std::ofstream::binary);
		SparseQuantizedMultiModTemplate* template1 = new SparseQuantizedMultiModTemplate();
		template1->deserialize(file_stream);
		file_stream.close ();
		return template1;
	}
}


void trainTemplate (const PointCloudXYZRGBA::ConstPtr & input, 
const std::vector<bool> &foreground_mask, pcl::LINEMOD & linemod)
{
  pcl::ColorGradientModality<pcl::PointXYZRGBA> color_grad_mod;
  color_grad_mod.setInputCloud (input);
  color_grad_mod.processInputData ();
  
  pcl::SurfaceNormalModality<pcl::PointXYZRGBA> surface_norm_mod;
  surface_norm_mod.setInputCloud (input);
  surface_norm_mod.processInputData ();

  std::vector<pcl::QuantizableModality*> modalities (2);
  modalities[0] = &color_grad_mod;
  modalities[1] = &surface_norm_mod;

  size_t min_x (input->width), min_y (input->height), max_x (0), max_y (0);
  pcl::MaskMap mask_map (input->width, input->height);
  for (size_t j = 0; j < input->height; ++j)
  {
    for (size_t i = 0; i < input->width; ++i)
    {
      mask_map (i,j) = foreground_mask[j*input->width+i];
      if (foreground_mask[j*input->width+i])
      {
        min_x = std::min (min_x, i);
        max_x = std::max (max_x, i);
        min_y = std::min (min_y, j);
        max_y = std::max (max_y, j);
      }
    }
  }

  std::vector<pcl::MaskMap*> masks (2);
  masks[0] = &mask_map;
  masks[1] = &mask_map;

  pcl::RegionXY region;
  region.x = static_cast<int> (min_x);
  region.y = static_cast<int> (min_y);
  region.width = static_cast<int> (max_x - min_x + 1);
  region.height = static_cast<int> (max_y - min_y + 1);

  printf ("%d %d %d %d\n", region.x, region.y, region.width, region.height);

  linemod.createAndAddTemplate (modalities, masks, region);
}


extern "C"
{
	int trainTemplatBytes (int w, int h, float* xyzrgb, int* mask, char*outbuf, int outlen)
	{ 
		PointCloudXYZRGBA::ConstPtr input = xyzrgb2cloud(w,h, xyzrgb);

		std::vector<bool> foreground_mask;
		foreground_mask.reserve(w*h);

		for(int i=0;i<w*h;i++)
			foreground_mask.push_back(mask[i]==1);

		pcl::LINEMOD linemod;
		trainTemplate(input, foreground_mask, linemod);
			
		std::stringstream ss;
		linemod.getTemplate(0).serialize(ss);
		int readlen = std::min<int>(outlen, ss.tellp());
		ss.read(outbuf, readlen);
		return readlen;
		
	}
	SparseQuantizedMultiModTemplate* trainTemplate (int w, int h, float* xyzrgb, int* mask)
	{ 
		PointCloudXYZRGBA::ConstPtr input = xyzrgb2cloud(w,h, xyzrgb);

		std::vector<bool> foreground_mask;
		foreground_mask.reserve(w*h);

		for(int i=0;i<w*h;i++)
			foreground_mask.push_back(mask[i]==1);

		pcl::LINEMOD linemod;
		trainTemplate(input, foreground_mask, linemod);
			
		std::stringstream ss;
		SparseQuantizedMultiModTemplate* newTemplate = new SparseQuantizedMultiModTemplate();
		linemod.getTemplate(0).serialize(ss);
		ss.seekg(std::ios_base::beg);
		newTemplate->deserialize(ss);
		return newTemplate;
		
	}

	SparseQuantizedMultiModTemplate** trainTemplates (int w, int h, float* xyzrgb, int* mask)
	{
		SparseQuantizedMultiModTemplate** templates = new SparseQuantizedMultiModTemplate*[1];
		templates[0] = trainTemplate(w,h,xyzrgb,mask);
		return templates;
	}

	int matchTemplatesBytes(int w, int h, float* xyzrgb, int nr_templates, char*buf, int buflen)
	{

		pcl::LINEMOD linemod;

		// Load the templates
		std::stringstream ss(std::string(buf,buf+buflen));
		for(int i=0;i<nr_templates;i++)
		{
			SparseQuantizedMultiModTemplate sqmmt;
			sqmmt.deserialize(ss);
			std::cout << "Loaded Template " << i << " #features x y w h " << sqmmt.features.size() << " " 
				<< sqmmt.region.x << " " << sqmmt.region.y << " " << sqmmt.region.width << " " << sqmmt.region.height << std::endl;
			linemod.addTemplate(sqmmt);
		}

		// Match the templates to the provided image
		const PointCloudXYZRGBA::Ptr input = xyzrgb2cloud(w,h, xyzrgb);
		std::vector<pcl::LINEMODDetection> detections = matchTemplates (input, linemod);

		
		//out of return space;
		if(sizeof(pcl::LINEMODDetection)*detections.size() > buflen)
			return -1;

		char* ptr = buf;
		for(int i=0;i<detections.size();i++)
		{
			memcpy(ptr, &detections[i], sizeof(pcl::LINEMODDetection));
			ptr += sizeof(pcl::LINEMODDetection);
			
		}
		return detections.size();
	}


	int matchTemplates(int w, int h, float* xyzrgb, int nr_templates, SparseQuantizedMultiModTemplate** templates, int debug)
	{
		const PointCloudXYZRGBA::Ptr input = xyzrgb2cloud(w,h, xyzrgb);
		pcl::LINEMOD linemod;

		// Load the templates
		for(int i=0;i<nr_templates;i++)
			linemod.addTemplate(*templates[i]);

		// Match the templates to the provided image
		std::vector<pcl::LINEMODDetection> detections = matchTemplates (input, linemod);

		if(debug)
		{
			// Output the position and score of the best match for each template
			for (size_t i = 0; i < detections.size (); ++i)
			{
				const LINEMODDetection & d = detections[i];
				printf ("%lu: %d %d %d %f\n", i, d.x, d.y, d.template_id, d.score);
			}

#ifdef WITH_PNG
			// Visualization code for testing purposes (requires libpng++)
			int i = 0;
			png::image<png::rgb_pixel> image (input->width, input->height);
			for (size_t y = 0; y < image.get_height (); ++y)
			{
				for (size_t x = 0; x < image.get_width (); ++x)
				{
					const pcl::PointXYZRGBA & p = input->points[i++];
					image[y][x] = png::rgb_pixel(p.r, p.g, p.b);
				}
			}
			// Draw a green box around the object
			for (size_t i = 0; i < detections.size (); ++i)
			{
				const LINEMODDetection & d = detections[i];

				const pcl::SparseQuantizedMultiModTemplate & tmplt = linemod.getTemplate (d.template_id);
				int w = tmplt.region.width;
				int h = tmplt.region.height;
				for (int x = d.x; x < d.x+w; ++x)
				{
					image[d.y][x] = png::rgb_pixel(0, 255, 0);
					image[d.y+h-1][x] = png::rgb_pixel(0, 255, 0);
				}
				for (int y = d.y; y < d.y+h; ++y)
				{
					image[y][d.x] = png::rgb_pixel(0, 255, 0);
					image[y][d.x+w-1] = png::rgb_pixel(0, 255, 0);
				}
			}
			image.write("output.png");
#endif
		}

		return detections.size();
	}
}


float* cloud2xyzrgb(PointCloudXYZRGBA::Ptr cloud)
{
	float* xyzrgb = new float[cloud->width*cloud->height*4];
	int iter=0;
	for(int i=0;i<cloud->points.size();i++)
	{
		xyzrgb[iter++] = cloud->points[i].x;
		xyzrgb[iter++] = cloud->points[i].y;
		xyzrgb[iter++] = cloud->points[i].z;
		xyzrgb[iter++] = cloud->points[i].rgb;
	}
	return xyzrgb;
}


int main(int argc, char**argv)
{
	if(argc!=3)
	{
		std::cout<<"argc = " << argc << std::endl;
		return -1;
	}
	// Load the input point cloud from the provided PCD file
	PointCloudXYZRGBA::Ptr cloud (new PointCloudXYZRGBA);
	if (loadPCDFile(argv[1], *cloud)<0) 
		return (-1);
	float* xyzrgb = cloud2xyzrgb(cloud);
	int debug=1;
	int nr_templates=0;
	SparseQuantizedMultiModTemplate **templates;
#if 1
	{
		int nr_templates1;
		SparseQuantizedMultiModTemplate** templates1 = loadTemplates(argv[2], &nr_templates1);

		int nr_batch=10;
		templates = new SparseQuantizedMultiModTemplate*[nr_templates1*nr_batch];
		nr_templates = nr_templates1*nr_batch;

		for(int i=0;i<nr_batch;i++)
		{
			for(int j=0;j< nr_templates1;j++)
				templates[i*nr_templates1+j]=templates1[j];
		}

		std::cout << nr_templates << " templates\n";
	}
#else
	nr_templates=1;
	templates = new SparseQuantizedMultiModTemplate*[nr_templates];;
	int mask[cloud->width*cloud->height];
	for(int i=0;i<cloud->width*cloud->height;i++)	
	{
		mask[i] = !isnan(xyzrgb[i*4]);
	}
	templates[0]= const_cast<SparseQuantizedMultiModTemplate*>(trainTemplate(cloud->width, cloud->height, xyzrgb, mask));
#endif
	{
		int nr_detection = matchTemplates(cloud->width, cloud->height, xyzrgb, nr_templates, templates, debug);
		std::cout << nr_detection << " detections\n";
	}
	delete[] templates;
	return 0;
}
