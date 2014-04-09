# caeplugin-CaeUnsRecomb2D
A Pointwise CAE plugin that merges 2D unstructured tri elements into quads based on a quality limit.

![CaeUnsRecomb2D Image][TriToQuadImage]

This experimental plugin converts unstructured 2D (surface) triangles to quads if a quality threashold is met. The transformed grid is saved as a Gmsh (msh) file.

Several solver attribute configuration settings are available to control export behavior.

## Building the Plugin
To build the CaeUnsRecomb2D plugin you must integrate this source code into your local PluginSDK installation by following these steps.

* Download and install the [Pointwise Plugin SDK][SDKdownload].
* Configure and validate the SDK following the [SDK's instructions][SDKdocs].
* Create an Recomb2D plugin project using the mkplugin script: `mkplugin -uns -cpp Recomb2D`
* Replace the project's generated files with the files from this repository.


## Disclaimer
Plugins are freely provided. They are not supported products of
Pointwise, Inc. Some plugins have been written and contributed by third
parties outside of Pointwise's control.

TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW, POINTWISE DISCLAIMS
ALL WARRANTIES, EITHER EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED
TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE, WITH REGARD TO THESE SCRIPTS. TO THE MAXIMUM EXTENT PERMITTED
BY APPLICABLE LAW, IN NO EVENT SHALL POINTWISE BE LIABLE TO ANY PARTY
FOR ANY SPECIAL, INCIDENTAL, INDIRECT, OR CONSEQUENTIAL DAMAGES
WHATSOEVER (INCLUDING, WITHOUT LIMITATION, DAMAGES FOR LOSS OF BUSINESS
INFORMATION, OR ANY OTHER PECUNIARY LOSS) ARISING OUT OF THE USE OF OR
INABILITY TO USE THESE SCRIPTS EVEN IF POINTWISE HAS BEEN ADVISED OF THE
POSSIBILITY OF SUCH DAMAGES AND REGARDLESS OF THE FAULT OR NEGLIGENCE OF
POINTWISE.

[TriToQuadImage]: https://raw.github.com/dbgarlisch/CaeUnsRecomb2D/master/triToQuad.png  "tris converted to quads Image"
[SDKdocs]: http://www.pointwise.com/plugins
[SDKdownload]: http://www.pointwise.com/plugins/#sdk_downloads
