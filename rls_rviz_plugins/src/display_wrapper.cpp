00001 /*
00002  * Copyright (c) 2009, Willow Garage, Inc.
00003  * All rights reserved.
00004  *
00005  * Redistribution and use in source and binary forms, with or without
00006  * modification, are permitted provided that the following conditions are met:
00007  *
00008  *     * Redistributions of source code must retain the above copyright
00009  *       notice, this list of conditions and the following disclaimer.
00010  *     * Redistributions in binary form must reproduce the above copyright
00011  *       notice, this list of conditions and the following disclaimer in the
00012  *       documentation and/or other materials provided with the distribution.
00013  *     * Neither the name of the Willow Garage, Inc. nor the names of its
00014  *       contributors may be used to endorse or promote products derived from
00015  *       this software without specific prior written permission.
00016  *
00017  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
00018  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
00019  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
00020  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
00021  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
00022  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
00023  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
00024  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
00025  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
00026  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
00027  * POSSIBILITY OF SUCH DAMAGE.
00028  */
00029 
00030 #include "display_wrapper.h"
00031 #include "display.h"
00032 #include "visualization_manager.h"
00033 #include "plugin/display_type_info.h"
00034 #include "plugin/plugin.h"
00035 #include "plugin/display_creator.h"
00036 #include "properties/property_manager.h"
00037 #include "properties/property.h"
00038 
00039 #include <ros/assert.h>
00040 
00041 #include <wx/confbase.h>
00042 
00043 namespace rviz
00044 {
00045 
00046 DisplayWrapper::DisplayWrapper(const std::string& package, const std::string& class_name, const PluginPtr& plugin, const std::string& name, VisualizationManager* manager)
00047 : manager_(manager)
00048 , name_(name)
  00049 , package_(package)
  00050 , class_name_(class_name)
  00051 , display_(0)
  00052 , property_manager_(0)
  00053 , enabled_(true)
  00054 {
  00055   manager->getDisplaysConfigLoadedSignal().connect(boost::bind(&DisplayWrapper::onDisplaysConfigLoaded, this, _1));
  00056   manager->getDisplaysConfigSavingSignal().connect(boost::bind(&DisplayWrapper::onDisplaysConfigSaved, this, _1));
00057 
  00058   if (plugin)
  00059   {
    00060     setPlugin(plugin);
    00061   }
 00062 }
00063 
00064 DisplayWrapper::~DisplayWrapper()
00065 {
  00066   destroyDisplay();
00067 
  00068   if (property_manager_)
  00069   {
    00070     property_manager_->deleteProperty(category_.lock());
    00071   }
 00072 }
00073 
00074 void DisplayWrapper::setName(const std::string& name)
00075 {
  00076   std::string old_name = name_;
  00077   name_ = name;
  00078   M_string new_props;
  00079   if (display_)
    00080   {
      00081     display_->setName(name);
00082 
  00083     if (config_)
  00084     {
    00085       wxString key;
    00086       long index;
    00087       bool cont = config_->GetFirstEntry(key, index);
    00088       while (cont)
		  00089       {
		    00090         wxString value;
		    00091         config_->Read(key, &value);
00092 
  00093         if (key.StartsWith(wxString::FromAscii((old_name + ".").c_str())))
  00094         {
    00095           wxString new_key = wxString::FromAscii(name.c_str()) + key.Mid(old_name.size() + 1, key.Length());
    00096           wxString val;
    00097           config_->Write(new_key, config_->Read(key, val));
    00098           config_->DeleteEntry(key);
00099 
  00100           std::string new_key_str = (const char*)new_key.mb_str();
 00101           std::string val_str = (const char*)val.mb_str();
 00102           new_props[new_key_str] = val_str;
 00103         }
00104 
  00105         cont = config_->GetNextEntry(key, index);
 00106       }
    00107     }
00108 
  00109     if (property_manager_)
  00110     {
    00111       property_manager_->changePrefix(old_name + ".", name + ".");
    00112     }
 00113   }
00114   else
    00115   {
      00116     M_string::iterator it = properties_.begin();
      00117     M_string::iterator end = properties_.end();
      00118     for (; it != end; ++it)
	00119     {
	  00120       const std::string& key = it->first;
	  00121       const std::string& val = it->second;
00122 
  00123       std::string new_key = name + "." + key.substr(old_name.size() + 1);
 00124       new_props[new_key] = val;
00125 
  00126       if (config_)
  00127       {
    00128         config_->DeleteEntry(wxString::FromAscii(key.c_str()));
    00129         config_->Write(wxString::FromAscii(new_key.c_str()), wxString::FromAscii(val.c_str()));
    00130       }
 00131     }
      00132   }
00133 
  00134   properties_ = new_props;
 00135 }
00136 
00137 void DisplayWrapper::setPlugin(const PluginPtr& plugin)
00138 {
  00139   ROS_ASSERT(!plugin_);
00140 
  00141   plugin_ = plugin;
00142 
  00143   if (plugin_)
  00144   {
    00145     plugin_->getLoadedSignal().connect(boost::bind(&DisplayWrapper::onPluginLoaded, this, _1));
    00146     plugin_->getUnloadingSignal().connect(boost::bind(&DisplayWrapper::onPluginUnloading, this, _1));
    00147   }
00148 
  00149   plugin_->autoLoad();
 00150   typeinfo_ = plugin_->getDisplayTypeInfo(class_name_);
00151 
  00152   if (typeinfo_)
  00153   {
    00154     // If the class name has been remapped, grab the new one
      00155     class_name_ = typeinfo_->class_name;
    00156   }
 00157 }
00158 
00159 void DisplayWrapper::loadProperties()
00160 {
  00161   if (!config_)
    00162   {
      00163     return;
      00164   }
00165 
  00166   properties_.clear();
00167 
  00168   wxString name;
 00169   long index;
 00170   bool cont = config_->GetFirstEntry(name, index);
 00171   while (cont)
	   00172   {
	     00173     wxString value;
	     00174     config_->Read(name, &value);
00175 
  00176     if (name.StartsWith(wxString::FromAscii((name_ + ".").c_str())))
  00177     {
    00178       properties_[(const char*)name.mb_str()] = (const char*)value.mb_str();
    00179     }
00180 
  00181     cont = config_->GetNextEntry(name, index);
 00182   }
 00183 }
00184 
00185 void DisplayWrapper::onDisplaysConfigLoaded(const boost::shared_ptr<wxConfigBase>& config)
00186 {
  00187   config_ = config;
00188 
  00189   loadProperties();
 00190 }
00191 
00192 void DisplayWrapper::onDisplaysConfigSaved(const boost::shared_ptr<wxConfigBase>& config)
00193 {
  00194   if (display_)
    00195   {
      00196     return;
      00197   }
00198 
  00199   M_string::iterator it = properties_.begin();
 00200   M_string::iterator end = properties_.end();
 00201   for (; it != end; ++it)
   00202   {
     00203     const std::string& name = it->first;
     00204     const std::string& value = it->second;
     00205     config->Write(wxString::FromAscii(name.c_str()), wxString::FromAscii(value.c_str()));
     00206   }
 00207 }
00208 
00209 bool DisplayWrapper::isLoaded() const
00210 {
  00211   return display_ != 0;
  00212 }
00213 
00214 void DisplayWrapper::createDisplay()
00215 {
  00216   if (!typeinfo_ || !typeinfo_->creator)
    00217   {
      00218     return;
      00219   }
00220 
  00221   if (display_)
  00222   {
    00223     return;
    00224   }
00225 
  00226   display_creating_(this);
00227 
  00228   display_ = plugin_->createDisplay(class_name_, name_, manager_);
 00229   if (display_)
   00230   {
     00231     if (property_manager_)
       00232     {
	 00233       display_->setPropertyManager(property_manager_, category_);
	 00234     }
00235 
  00236     display_created_(this);
 00237   }
 00238 }
00239 
00240 void DisplayWrapper::destroyDisplay()
00241 {
  00242   if (display_)
    00243   {
      00244     display_destroying_(this);
00245 
  00246     display_->disable(false);
 00247     delete display_;
 00248     display_ = 0;
00249 
  00250     display_destroyed_(this);
 00251   }
  00252 }
00253 
00254 void DisplayWrapper::onPluginLoaded(const PluginStatus& st)
00255 {
  00256   ROS_ASSERT(st.plugin == plugin_.get());
  00257   ROS_ASSERT(display_ == 0);
00258 
  00259   createDisplay();
00260 
  00261   if (display_)
  00262   {
    00263     display_->setEnabled(enabled_, true);
    00264   }
 00265 }
00266 
00267 void DisplayWrapper::onPluginUnloading(const PluginStatus& st)
00268 {
  00269   ROS_ASSERT(st.plugin == plugin_.get());
  00270   ROS_ASSERT(display_ != 0);
00271 
  00272   loadProperties();
 00273   destroyDisplay();
 00274 }
00275 
00276 bool DisplayWrapper::isEnabled()
00277 {
  00278   if (display_)
    00279   {
      00280     return display_->isEnabled();
      00281   }
00282 
  00283   return enabled_;
 00284 }
00285 
00286 void DisplayWrapper::setEnabled(bool enabled)
00287 {
  00288   if (display_)
    00289   {
      00290     display_->setEnabled(enabled, false);
      00291   }
00292 
  00293   enabled_ = enabled;
00294 
  00295   propertyChanged(category_);
 00296 }
00297 
00298 void DisplayWrapper::setPropertyManager(PropertyManager* property_manager, const CategoryPropertyWPtr& parent)
00299 {
  00300   ROS_ASSERT(!property_manager_);
00301 
  00302   property_manager_ = property_manager;
00303 
  00304   category_ = property_manager_->createCheckboxCategory( getName(), "Enabled", getName() + ".", boost::bind( &DisplayWrapper::isEnabled, this ),
								 00305                                                          boost::bind( &DisplayWrapper::setEnabled, this, _1 ), parent, this );
 00306   setPropertyHelpText(category_, typeinfo_->help_description);
00307 
  00308   if (display_)
  00309   {
    00310     display_->setPropertyManager(property_manager, category_);
    00311   }
 00312 }
00313 
00314 } // namespace rviz
